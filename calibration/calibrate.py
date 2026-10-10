"""Phototransistor calibration for the archerfish head.

Upload calibration/calibration.ino first, then run:

    pip install -r calibration/requirements.txt
    python calibration/calibrate.py --port COM3

The script walks through three measurements:
  1. ambient - LED off, normal room light          -> per-sensor offset
  2. center  - LED on, straight ahead, working distance -> per-sensor gain
  3. far     - LED on, straight ahead, farthest distance -> MIN_BRIGHTNESS

It prints C++ constants to paste into tracking_mechanism.ino and saves the
raw samples to calibration/data/ so they can be reused for characterization.
"""

import argparse
import csv
import math
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError:
    sys.exit("pyserial is not installed: pip install -r calibration/requirements.txt")

SENSORS = ["tl", "tr", "bl", "br"]
ADC_MAX = 1023
SATURATION = 900         # readings this high are near the 1023 ceiling and carry little direction information
MIN_SIGNAL = 20          # LED signal above ambient below this is too weak to calibrate
DATA_DIR = Path(__file__).resolve().parent / "data"


DEFAULT_HEADER = ["ms"] + SENSORS   # column order printed by calibration.ino


class LineReader(threading.Thread):
    """Reads serial lines continuously in the background.

    Some boards (e.g. the UNO R4 WiFi) stop streaming if the computer stops reading
    for a few seconds, which happens while the script waits at a prompt. Reading
    all the time keeps the stream alive; record() just picks the lines it needs.
    """

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.lines = []          # (time received, line)
        self.error = None
        self.running = True

    def run(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as e:
                self.error = e
                return
            if line:
                with self.lock:
                    self.lines.append((time.time(), line))
                    del self.lines[:-2000]   # keep memory bounded between steps

    def since(self, start):
        with self.lock:
            return [line for t, line in self.lines if t >= start]

    def stop(self):
        self.running = False
        self.join(timeout=2)
        self.ser.close()


def open_port(port, baud):
    """Open the serial port, start the background reader, and work out the CSV columns.

    Boards with an Uno-style USB chip reset when the port opens and print the header.
    Native-USB boards (shown as "USB Serial Device") don't reset, so the header was
    printed long ago; for those, recognize calibration.ino's data lines directly.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\nCheck the port and close the Serial Monitor.")
    reader = LineReader(ser)
    reader.start()
    start = time.time()
    while time.time() < start + 5 and not reader.error:
        time.sleep(0.1)
        for line in reader.since(start):
            if line.startswith("ms,"):
                if line.split(",") != DEFAULT_HEADER:
                    sys.exit(f"Header {line} is not from calibration.ino (expected "
                             f"{','.join(DEFAULT_HEADER)}). Upload calibration/calibration.ino and try again.")
                return reader, DEFAULT_HEADER
            parts = line.split(",")
            if len(parts) == len(DEFAULT_HEADER) and all(p.isdigit() for p in parts):
                return reader, DEFAULT_HEADER
    received = reader.since(start)
    if received:
        sys.exit("Got serial data, but not from calibration.ino. Last lines received:\n  "
                 + "\n  ".join(received[-5:]) + "\nUpload calibration/calibration.ino and try again.")
    sys.exit(f"No data on {port}. Check the port, the baud rate (115200), and that the "
             f"Serial Monitor is closed.")


LOST_CONNECTION = ("Lost the connection to the Arduino: it was unplugged, reset, or the port was "
                   "taken by another program (close the Serial Monitor).")


def record(reader, header, seconds):
    """Collect samples for a number of seconds, returning a list of dicts of sensor readings."""
    start = time.time()
    while time.time() < start + seconds and not reader.error:
        time.sleep(0.1)
    if reader.error:
        sys.exit(f"{LOST_CONNECTION}\n({reader.error})")
    received = reader.since(start)
    samples = []
    for line in received:
        parts = line.split(",")
        if len(parts) != len(header):
            continue
        try:
            samples.append(dict(zip(header, (int(p) for p in parts))))
        except ValueError:
            continue
    if not samples:
        if not received:
            sys.exit(f"No data for {seconds} s. The Arduino stopped sending.\n{LOST_CONNECTION}")
        sys.exit("Got serial data, but not from calibration.ino. Last lines received:\n  "
                 + "\n  ".join(received[-5:]) + "\nUpload calibration/calibration.ino and try again.")
    return samples


def save(samples, header, stamp, step):
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"{stamp}_{step}.csv"
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(samples)
    return path


def summarize(samples, step):
    """Print and return per-sensor mean, with saturation warnings."""
    means = {}
    print(f"  {len(samples)} samples")
    for s in SENSORS:
        vals = [row[s] for row in samples]
        means[s] = statistics.mean(vals)
        sd = statistics.pstdev(vals)
        print(f"  {s}: mean {means[s]:7.1f}  sd {sd:5.1f}  min {min(vals):4d}  max {max(vals):4d}")
        if max(vals) >= SATURATION:
            print(f"  WARNING: {s} is saturating during '{step}'. Use a smaller pull-down "
                  f"resistor or move the LED farther away.")
    return means


def calibrated(row, offsets, gains):
    return {s: max(0.0, (row[s] - offsets[s]) * gains[s]) for s in SENSORS}


def diffs(c):
    """Same math as diffSenseNorm() in tracking_mechanism.ino."""
    total = sum(c.values())
    if total == 0:
        return 0.0, 0.0
    x = ((c["tl"] + c["bl"]) - (c["tr"] + c["br"])) / total
    y = ((c["tl"] + c["tr"]) - (c["bl"] + c["br"])) / total
    return x, y


def percentile(vals, p):
    vals = sorted(vals)
    return vals[min(len(vals) - 1, int(p * len(vals)))]


def prompt(msg):
    input(f"\n{msg}\nPress Enter when ready...")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="serial port, e.g. COM3 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=5, help="recording time per step")
    parser.add_argument("--skip-far", action="store_true", help="skip the far-distance step")
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    reader, header = open_port(args.port, args.baud)
    print("Connected. The head is held at home (pan 90, tilt 90).")

    # 1. ambient offsets
    prompt("STEP 1 - AMBIENT: turn the LED off. Leave room lights as they will be during the demo.")
    ambient = record(reader, header, args.seconds)
    print(f"  saved {save(ambient, header, stamp, 'ambient')}")
    offsets = summarize(ambient, "ambient")

    # 2. centered gains
    prompt("STEP 2 - CENTER: turn the LED on and place it straight in front of the head\n"
           "at the normal working distance, centered on the sensor square.")
    center = record(reader, header, args.seconds)
    print(f"  saved {save(center, header, stamp, 'center')}")
    center_means = summarize(center, "center")

    if max(center_means.values()) >= SATURATION:
        sys.exit(f"\nThe sensors read near the maximum ({ADC_MAX}) at center, so the gains would be wrong.\n"
                 f"Move the LED farther away (aim for center readings of about 300-700) and run again.")
    signal = {s: center_means[s] - offsets[s] for s in SENSORS}
    for s in SENSORS:
        if signal[s] < MIN_SIGNAL:
            sys.exit(f"\n{s} only rises {signal[s]:.1f} above ambient. Check its wiring, baffle "
                     f"and LED alignment, then run again.")
    target = statistics.mean(signal.values())
    gains = {s: target / signal[s] for s in SENSORS}

    # residual error at center after calibration, used for FOUND_THRESHOLD
    errors = []
    for row in center:
        x, y = diffs(calibrated(row, offsets, gains))
        errors.append(math.hypot(x, y))
    found_threshold = 1.5 * percentile(errors, 0.99)

    # 3. far distance for MIN_BRIGHTNESS
    ambient_totals = [sum(calibrated(r, offsets, gains).values()) for r in ambient]
    min_brightness = None
    if not args.skip_far:
        prompt("STEP 3 - FAR: keep the LED centered but move it to the farthest distance\n"
               "the fish needs to track.")
        far = record(reader, header, args.seconds)
        print(f"  saved {save(far, header, stamp, 'far')}")
        summarize(far, "far")
        far_totals = [sum(calibrated(r, offsets, gains).values()) for r in far]
        lo, hi = max(ambient_totals), min(far_totals)
        if hi <= lo:
            print(f"\n  WARNING: at the far distance the LED (min total {hi:.0f}) is not brighter than "
                  f"ambient (max total {lo:.0f}). Shorten the range or use a brighter LED.")
        min_brightness = round((lo + hi) / 2)

    reader.stop()

    # results
    lines = [
        "// calibration from calibration/calibrate.py, " + stamp,
        "const float OFF_TL = {tl:.1f}, OFF_TR = {tr:.1f}, OFF_BL = {bl:.1f}, OFF_BR = {br:.1f};".format(**offsets),
        "const float GAIN_TL = {tl:.3f}, GAIN_TR = {tr:.3f}, GAIN_BL = {bl:.3f}, GAIN_BR = {br:.3f};".format(**gains),
    ]
    print("\n" + "=" * 70)
    print("Paste into tracking_mechanism.ino, replacing the OFF_ and GAIN_ lines:\n")
    print("\n".join(lines))
    print(f"\nSuggested FOUND_THRESHOLD: {found_threshold:.4f}"
          f"  (1.5 x the 99th percentile error with the LED centered)")
    if min_brightness is not None:
        print(f"Suggested MIN_BRIGHTNESS: {min_brightness}"
              f"  (midway between max ambient total and min far total)")
    print("\nThese suggestions only cover noise at center. Check them against your angle and")
    print("distance characterization before using them as final values.")

    summary = DATA_DIR / f"{stamp}_constants.txt"
    with open(summary, "w") as f:
        f.write("\n".join(lines) + "\n")
        f.write(f"// suggested FOUND_THRESHOLD = {found_threshold:.4f}\n")
        if min_brightness is not None:
            f.write(f"// suggested MIN_BRIGHTNESS = {min_brightness}\n")
    print(f"\nSaved {summary}\n")

    # write calibration_values.h into the sketches
    import apply
    apply.apply(stamp)


if __name__ == "__main__":
    main()
