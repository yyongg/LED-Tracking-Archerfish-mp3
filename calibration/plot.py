"""Plot a calibration run: python calibration/plot.py [--run 20261008_211521] [--save]

Figure 1: readings over time per step, raw (top) and calibrated (bottom).
Figure 2: center-step pointing error before and after calibration.
"""

import argparse
import csv
import math
import statistics

import matplotlib.pyplot as plt

from calibrate import DATA_DIR, SENSORS, calibrated, diffs, percentile

STEPS = ["ambient", "center", "far"]


def load(stamp, step):
    with open(DATA_DIR / f"{stamp}_{step}.csv", newline="") as f:
        return [{k: int(v) for k, v in row.items()} for row in csv.DictReader(f)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run")
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()
    stamp = args.run or sorted(DATA_DIR.glob("*_ambient.csv"))[-1].name.removesuffix("_ambient.csv")
    data = {step: load(stamp, step) for step in STEPS}

    # same math as calibrate.py
    offsets = {s: statistics.mean(r[s] for r in data["ambient"]) for s in SENSORS}
    signal = {s: statistics.mean(r[s] for r in data["center"]) - offsets[s] for s in SENSORS}
    gains = {s: statistics.mean(signal.values()) / signal[s] for s in SENSORS}

    # figure 1: readings over time, raw and calibrated
    fig1, axes = plt.subplots(2, len(STEPS), figsize=(15, 7), sharex="col")
    fig1.suptitle(f"Calibration run {stamp}")
    for col, step in enumerate(STEPS):
        samples = data[step]
        t = [(r["ms"] - samples[0]["ms"]) / 1000 for r in samples]
        cal = [calibrated(r, offsets, gains) for r in samples]
        for s in SENSORS:
            axes[0][col].plot(t, [r[s] for r in samples], label=s)
            axes[1][col].plot(t, [c[s] for c in cal], label=s)
        axes[0][col].set_title(f"{step} - raw")
        axes[1][col].set_title(f"{step} - calibrated")
        axes[1][col].set_xlabel("time (s)")
    axes[0][0].legend()
    fig1.tight_layout()

    # figure 2: pointing error at center, before and after calibration
    raw_xy = [diffs({s: r[s] for s in SENSORS}) for r in data["center"]]
    cal_xy = [diffs(calibrated(r, offsets, gains)) for r in data["center"]]
    threshold = 1.5 * percentile([math.hypot(x, y) for x, y in cal_xy], 0.99)
    fig2, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(*zip(*raw_xy), s=10, color="tab:gray", label="raw")
    ax.scatter(*zip(*cal_xy), s=10, color="tab:blue", label="calibrated")
    ax.add_patch(plt.Circle((0, 0), threshold, fill=False, color="tab:red", linestyle="--",
                            label=f"suggested FOUND_THRESHOLD {threshold:.4f}"))
    lim = max(abs(v) for xy in raw_xy + cal_xy for v in xy) * 1.2
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.axvline(0, color="black", linewidth=0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x_diff  (+ = LED to the left)")
    ax.set_ylabel("y_diff  (+ = LED above)")
    ax.legend()
    fig2.tight_layout()

    if args.save:
        fig1.savefig(DATA_DIR / f"{stamp}_readings.png", dpi=150)
        fig2.savefig(DATA_DIR / f"{stamp}_center_error.png", dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
