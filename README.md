# LED-Tracking Archerfish

Archie is a robotic archerfish built for Principles of Integrated Engineering (PIE), Mini-Project 3. A glowing "bug" (an LED on a motor-driven wheel) sits in front of him. Archie turns his head on a pan-tilt mechanism to aim at it. When he's locked on, he "spits" by firing a laser diode for one second. The startled bug then spins off to a new spot on the wheel, and Archie starts aiming again.

Demo video: https://www.youtube.com/watch?v=U1OvieUQeXw

## How it works

- **Sensing:** Four phototransistors sit in a 2×2 holder in the fish's mouth, with walls between them. Each reading is calibrated with `(raw - OFF) * GAIN`. The four readings are then combined into normalized left/right and up/down differences.
- **Tracking:** Each loop, the pan and tilt servos step toward the brighter side by `round(30 × diff)`, limited to 1–5°, then wait 60 ms. Large errors produce big steps and small errors produce careful 1° steps.
- **Waiting:** If the summed reading is below `MIN_BRIGHTNESS`, the LED isn't visible and the head holds still.
- **Spitting:** Once both differences are below `FOUND_THRESHOLD = 0.02`, the head tilts up 10° to correct the laser's aim. The laser then fires for 1 s and the wheel motor runs for a random 1–2000 ms. If a servo limit stops the head short of the bug, Archie fires from there anyway. The next round starts when you press Enter in the Serial Monitor.

## Repository layout

```
tracking_mechanism/   Main firmware the fish runs during the demo
tests/mechanism_test/ Serial-driven test sketch for each subsystem (send '?' for commands)
calibration/
  calibration.ino     Streams the four raw sensor readings at 115200 baud
  calibrate.py        Guided ambient / center / far calibration; saves raw CSVs to data/
  apply.py            Writes offsets, gains and MIN_BRIGHTNESS into each sketch's calibration_values.h
  plot.py             Plots a run's raw vs. calibrated readings and pointing error
  data/               Logged calibration runs
```

## Hardware

- Arduino UNO R4 WiFi + Adafruit Motor Shield v2.3
- 4 phototransistors on A0–A3 (TL, TR, BL, BR), each with a 100 kΩ pull-down resistor
- Pan servo on pin 9, tilt servo on pin 10
- 5 mW 650 nm laser diode on pin 8 (with a 100 Ω resistor)
- Gearmotor on shield port M4, driving the LED wheel
- 3D-printed PLA fish (6 parts, about 130 g) inside a laser-cut plywood box

## Usage

**1. Calibrate** (needs Python with `pip install -r calibration/requirements.txt`):

1. Upload `calibration/calibration.ino`.
2. Run `python calibration/calibrate.py --port COM3` and follow the prompts. You'll record three steps: LED off, LED centered at the working distance, and LED centered at the far distance.
3. The script updates `calibration_values.h` in both sketches automatically. To re-apply an older run, use `python calibration/apply.py --run <timestamp>`. To view a run, use `python calibration/plot.py [--run <timestamp>] [--save]`.

**2. Run:** Install the `Servo` and `Adafruit Motor Shield V2` libraries, then upload `tracking_mechanism/tracking_mechanism.ino`. Open the Serial Monitor at 115200 baud. Each loop logs one CSV line with the time, raw readings, differences, servo angles and status flags. Press Enter after each shot to start the next round.

## Known limitations

- Calibration is exact at only one distance. At the far distance, a centered LED still reads `x_diff ≈ 0.05`, so Archie settles slightly off-center.
- Tracking is slow, because the head waits 60 ms per step. The laser and motor use `delay()`, which leaves Archie blind for up to about 3 s after each shot.
