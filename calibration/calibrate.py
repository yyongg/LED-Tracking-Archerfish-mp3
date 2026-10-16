"""Phototransistor calibration. Upload calibration.ino, then: python calibrate.py --port COM3

Steps: ambient (LED off) -> offsets, center (LED on, ahead) -> gains, far -> MIN_BRIGHTNESS.
Saves raw samples to data/ and writes calibration_values.h into the sketches.
"""

import argparse
import csv
import math
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path

import serial

import apply

SENSORS = ["tl", "tr", "bl", "br"]
HEADER = ["ms"] + SENSORS   # column order printed by calibration.ino
DATA_DIR = Path(__file__).resolve().parent / "data"


class LineReader(threading.Thread):
    """Reads serial lines nonstop, since some boards stop streaming if nobody reads."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lines = []   # (time received, line)

    def run(self):
        while True:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line:
                self.lines.append((time.time(), line))
                del self.lines[:-2000]

    def since(self, start):
        return [line for t, line in list(self.lines) if t >= start]


def record(reader, seconds):
    """Collect samples for a number of seconds, as a list of dicts of readings."""
    start = time.time()
    time.sleep(seconds)
    samples = []
    for line in reader.since(start):
        parts = line.split(",")
        if len(parts) == len(HEADER) and all(p.isdigit() for p in parts):
            samples.append(dict(zip(HEADER, map(int, parts))))
    return samples


def step(reader, seconds, stamp, name, msg):
    input(f"\n{msg} Press Enter...")
    samples = record(reader, seconds)
    DATA_DIR.mkdir(exist_ok=True)
    with open(DATA_DIR / f"{stamp}_{name}.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=HEADER)
        writer.writeheader()
        writer.writerows(samples)
    means = {s: statistics.mean(row[s] for row in samples) for s in SENSORS}
    print(f"  {len(samples)} samples, means " + "  ".join(f"{s} {means[s]:.0f}" for s in SENSORS))
    return samples, means


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=5)
    args = parser.parse_args()

    reader = LineReader(serial.Serial(args.port, 115200, timeout=1))
    reader.start()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    ambient, offsets = step(reader, args.seconds, stamp, "ambient", "AMBIENT: LED off.")
    center, center_means = step(reader, args.seconds, stamp, "center", "CENTER: LED on, straight ahead.")
    signal = {s: center_means[s] - offsets[s] for s in SENSORS}
    gains = {s: statistics.mean(signal.values()) / signal[s] for s in SENSORS}

    # sensor noise at center, a starting point for FOUND_THRESHOLD
    errors = [math.hypot(*diffs(calibrated(row, offsets, gains))) for row in center]
    found_threshold = 1.5 * percentile(errors, 0.99)

    # MIN_BRIGHTNESS: midway between the brightest ambient total and the dimmest far total
    far, _ = step(reader, args.seconds, stamp, "far", "FAR: LED on, farthest distance.")
    ambient_max = max(sum(calibrated(r, offsets, gains).values()) for r in ambient)
    far_min = min(sum(calibrated(r, offsets, gains).values()) for r in far)
    min_brightness = round((ambient_max + far_min) / 2)

    text = "\n".join([
        "// calibration from calibration/calibrate.py, " + stamp,
        "const float OFF_TL = {tl:.1f}, OFF_TR = {tr:.1f}, OFF_BL = {bl:.1f}, OFF_BR = {br:.1f};".format(**offsets),
        "const float GAIN_TL = {tl:.3f}, GAIN_TR = {tr:.3f}, GAIN_BL = {bl:.3f}, GAIN_BR = {br:.3f};".format(**gains),
        f"// suggested FOUND_THRESHOLD = {found_threshold:.4f}",
        f"// suggested MIN_BRIGHTNESS = {min_brightness}",
    ]) + "\n"
    (DATA_DIR / f"{stamp}_constants.txt").write_text(text)
    print("\n" + text)
    apply.apply(stamp)


if __name__ == "__main__":
    main()
