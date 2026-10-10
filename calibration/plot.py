"""Plot a calibration run saved by calibrate.py.

    python calibration/plot.py                    # latest run in calibration/data/
    python calibration/plot.py --run 20261008_211521
    python calibration/plot.py --save             # also save PNGs next to the data

Figure 1: each step's readings over time, raw (top) and calibrated (bottom).
          After calibration the four sensors should overlap in the center step.
Figure 2: x_diff / y_diff for the center step, before and after calibration.
          Calibrated points should cluster on (0, 0), inside the suggested
          FOUND_THRESHOLD circle.

Offsets and gains are computed from the saved data with the same math as calibrate.py.
"""

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

import matplotlib.pyplot as plt

from calibrate import DATA_DIR, SENSORS, calibrated, diffs, percentile

STEPS = ["ambient", "center", "far"]
COLORS = {"tl": "tab:blue", "tr": "tab:orange", "bl": "tab:green", "br": "tab:red"}


def load(path):
    with open(path, newline="") as f:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(f)]


def latest_run():
    stamps = sorted({p.name.rsplit("_", 1)[0] for p in DATA_DIR.glob("*_ambient.csv")})
    if not stamps:
        sys.exit(f"No calibration runs in {DATA_DIR}. Run calibrate.py first.")
    return stamps[-1]


def seconds(samples):
    t0 = samples[0]["ms"]
    return [(r["ms"] - t0) / 1000 for r in samples]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", help="run timestamp, e.g. 20261008_211521 (default: latest)")
    parser.add_argument("--save", action="store_true", help="save PNGs to calibration/data/")
    args = parser.parse_args()

    stamp = args.run or latest_run()
    data = {}
    for step in STEPS:
        path = DATA_DIR / f"{stamp}_{step}.csv"
        if path.exists():
            data[step] = load(path)
    if "ambient" not in data:
        sys.exit(f"No ambient data for run {stamp}.")
    print(f"run {stamp}: " + ", ".join(f"{s} ({len(d)} samples)" for s, d in data.items()))

    # same math as calibrate.py
    offsets = {s: statistics.mean(r[s] for r in data["ambient"]) for s in SENSORS}
    gains = {s: 1.0 for s in SENSORS}
    if "center" in data:
        signal = {s: statistics.mean(r[s] for r in data["center"]) - offsets[s] for s in SENSORS}
        if min(signal.values()) > 0:
            target = statistics.mean(signal.values())
            gains = {s: target / signal[s] for s in SENSORS}
        else:
            print("WARNING: a sensor did not rise above ambient in the center step; using gain 1.0")
    else:
        print("No center step in this run, so gains are 1.0 (offsets only).")
    print("offsets: " + ", ".join(f"{s} {offsets[s]:.1f}" for s in SENSORS))
    print("gains:   " + ", ".join(f"{s} {gains[s]:.3f}" for s in SENSORS))

    # figure 1: readings over time, raw and calibrated
    steps = list(data)
    fig1, axes = plt.subplots(2, len(steps), figsize=(5 * len(steps), 7), squeeze=False, sharex="col")
    fig1.suptitle(f"Calibration run {stamp}: sensor readings")
    for col, step in enumerate(steps):
        samples = data[step]
        t = seconds(samples)
        cal = [calibrated(r, offsets, gains) for r in samples]
        for s in SENSORS:
            axes[0][col].plot(t, [r[s] for r in samples], color=COLORS[s], label=s)
            axes[1][col].plot(t, [c[s] for c in cal], color=COLORS[s], label=s)
        axes[0][col].set_title(f"{step} - raw")
        axes[1][col].set_title(f"{step} - calibrated")
        axes[1][col].set_xlabel("time (s)")
    axes[0][0].set_ylabel("ADC counts (0-1023)")
    axes[1][0].set_ylabel("calibrated counts")
    axes[0][0].legend(loc="upper right")
    fig1.tight_layout()

    figs = {"readings": fig1}

    # figure 2: pointing error at center, before and after calibration
    if "center" in data:
        no_cal_off = {s: 0.0 for s in SENSORS}
        no_cal_gain = {s: 1.0 for s in SENSORS}
        raw_xy = [diffs(calibrated(r, no_cal_off, no_cal_gain)) for r in data["center"]]
        cal_xy = [diffs(calibrated(r, offsets, gains)) for r in data["center"]]
        threshold = 1.5 * percentile([math.hypot(x, y) for x, y in cal_xy], 0.99)

        fig2, ax = plt.subplots(figsize=(6, 6))
        ax.scatter(*zip(*raw_xy), s=10, alpha=0.5, color="tab:gray", label="raw")
        ax.scatter(*zip(*cal_xy), s=10, alpha=0.7, color="tab:blue", label="calibrated")
        ax.add_patch(plt.Circle((0, 0), threshold, fill=False, color="tab:red", linestyle="--",
                                label=f"suggested FOUND_THRESHOLD {threshold:.4f}"))
        lim = max(0.05, *(abs(v) for xy in raw_xy + cal_xy for v in xy)) * 1.2
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.axhline(0, color="black", linewidth=0.5)
        ax.axvline(0, color="black", linewidth=0.5)
        ax.set_aspect("equal")
        ax.set_xlabel("x_diff  (+ = LED to the left)")
        ax.set_ylabel("y_diff  (+ = LED above)")
        ax.set_title(f"LED centered: pointing error, run {stamp}")
        ax.legend(loc="upper right")
        fig2.tight_layout()
        figs["center_error"] = fig2

        raw_mean = (statistics.mean(x for x, _ in raw_xy), statistics.mean(y for _, y in raw_xy))
        cal_mean = (statistics.mean(x for x, _ in cal_xy), statistics.mean(y for _, y in cal_xy))
        print(f"center mean error  raw: x {raw_mean[0]:+.4f} y {raw_mean[1]:+.4f}"
              f"   calibrated: x {cal_mean[0]:+.4f} y {cal_mean[1]:+.4f}")

    if args.save:
        for name, fig in figs.items():
            path = DATA_DIR / f"{stamp}_{name}.png"
            fig.savefig(path, dpi=150)
            print(f"saved {path}")

    plt.show()


if __name__ == "__main__":
    main()
