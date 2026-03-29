#!/usr/bin/env python3
"""
HEF Flight Log Plotter — Generic CSV flight data visualization.

Usage:
    python plot_flight.py <csv_file> [--cols COL1,COL2,...] [--skip COL1,COL2,...]

Reads any HEF flight log CSV and plots all numeric columns as subplots
against frame number. String/key columns are shown as event markers.

Examples:
    python plot_flight.py HeliFlightLog_1774789594550.csv
    python plot_flight.py HeliFlightLog_1774789594550.csv --cols torqueX,torqueY,torqueZ,omegaX,omegaY,omegaZ
    python plot_flight.py HeliFlightLog_1774789594550.csv --skip simPosX,simPosZ
"""

import sys
import csv
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


def load_csv(path):
    """Load CSV, return headers and column data (numeric as float, else string).
    Handles duplicate column names by appending _2, _3, etc."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        raw_headers = next(reader)
        raw_rows = list(reader)

    # Deduplicate headers
    seen = {}
    headers = []
    for h in raw_headers:
        if h in seen:
            seen[h] += 1
            headers.append(f"{h}_{seen[h]}")
        else:
            seen[h] = 1
            headers.append(h)

    columns = {h: [] for h in headers}
    for row in raw_rows:
        for h, val in zip(headers, row):
            try:
                columns[h].append(float(val))
            except ValueError:
                columns[h].append(val)

    return headers, columns


def classify_columns(headers, columns):
    """Split columns into numeric and string categories."""
    numeric = []
    string_cols = []
    for h in headers:
        if not columns[h]:
            continue
        if isinstance(columns[h][0], float):
            numeric.append(h)
        else:
            string_cols.append(h)
    return numeric, string_cols


def group_columns(numeric_cols):
    """Group related columns for shared subplots."""
    groups = []
    used = set()

    # Define grouping rules: columns sharing a prefix or known groups
    known_groups = [
        (["torqueX", "torqueY", "torqueZ"], "Torque (Nm)"),
        (["omegaX", "omegaY", "omegaZ"], "Angular Velocity (deg/s)"),
        (["desAngleX", "desAngleY", "desAngleZ"], "Desired Angles (deg)"),
        (["actAngleX", "actAngleY", "actAngleZ"], "Actual Angles (deg)"),
        (["desiredVelX", "desiredVelZ"], "Desired Vel XZ (m/s)"),
        (["simVelX", "simVelZ"], "Sim Vel XZ (m/s)"),
        (["simPosX", "simPosZ"], "Sim Pos XZ"),
        (["errX", "errZ"], "Pos Error XZ (m)"),
        (["errRateX", "errRateZ"], "Error Rate XZ"),
        (["velocityX", "velocityY", "velocityZ"], "Bullet Velocity (m/s)"),
        (["fwdX", "fwdZ"], "Forward Dir"),
        (["angleZ", "angleX"], "Body Angles (rad)"),
        (["actualX", "actualZ"], "Position XZ"),
    ]

    for cols, label in known_groups:
        present = [c for c in cols if c in numeric_cols and c not in used]
        if present:
            groups.append((present, label))
            used.update(present)

    # Remaining numeric columns get individual plots
    for c in numeric_cols:
        if c not in used:
            groups.append(([c], c))
            used.add(c)

    return groups


def detect_key_events(columns, headers):
    """Extract key press change events from the 'keys' column."""
    if "keys" not in headers:
        return []
    keys_col = columns["keys"]
    events = []
    prev = ""
    for i, k in enumerate(keys_col):
        if k != prev and k != "-":
            events.append((i, str(k)))
        prev = k
    return events


def plot_flight(path, include_cols=None, skip_cols=None):
    headers, columns = load_csv(path)
    numeric_cols, string_cols = classify_columns(headers, columns)

    # Filter columns
    if include_cols:
        include_set = set(include_cols)
        numeric_cols = [c for c in numeric_cols if c in include_set]
    if skip_cols:
        skip_set = set(skip_cols)
        numeric_cols = [c for c in numeric_cols if c not in skip_set]

    # Always skip 'frame' and 'ms' from plots (used as x-axis or metadata)
    numeric_cols = [c for c in numeric_cols if c not in ("frame", "ms")]

    groups = group_columns(numeric_cols)
    if not groups:
        print("No numeric columns to plot.")
        return

    n_frames = len(columns[headers[0]])
    frames = np.arange(1, n_frames + 1)

    # Key events for vertical markers
    key_events = detect_key_events(columns, headers)

    # Color cycle for multi-line subplots
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    fig, axes = plt.subplots(len(groups), 1, figsize=(16, max(3 * len(groups), 6)),
                             sharex=True, squeeze=False)
    axes = axes.flatten()

    for idx, (cols, label) in enumerate(groups):
        ax = axes[idx]
        for ci, col in enumerate(cols):
            data = np.array(columns[col], dtype=float)
            c = colors[ci % len(colors)]
            ax.plot(frames, data, label=col, color=c, linewidth=0.8)
        ax.set_ylabel(label, fontsize=8)
        ax.legend(fontsize=7, loc="upper right", ncol=min(len(cols), 4))
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)

        # Key event markers (subtle)
        for frame_idx, key_str in key_events:
            ax.axvline(frame_idx + 1, color="gray", alpha=0.15, linewidth=0.5)

    # Key event labels on top subplot only
    if key_events and len(axes) > 0:
        ax_top = axes[0]
        for frame_idx, key_str in key_events:
            ax_top.annotate(key_str, xy=(frame_idx + 1, ax_top.get_ylim()[1]),
                           fontsize=5, alpha=0.6, rotation=90, va="top",
                           color="gray")

    axes[-1].set_xlabel("Frame", fontsize=9)

    title = Path(path).stem
    fig.suptitle(title, fontsize=11, y=0.995)
    fig.tight_layout(rect=[0, 0, 1, 0.99])

    # Save as PNG next to the CSV
    out_path = Path(path).with_suffix(".png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="HEF Flight Log Plotter")
    parser.add_argument("csv_file", help="Path to HeliFlightLog CSV")
    parser.add_argument("--cols", help="Comma-separated columns to include (default: all)")
    parser.add_argument("--skip", help="Comma-separated columns to skip")
    args = parser.parse_args()

    include = args.cols.split(",") if args.cols else None
    skip = args.skip.split(",") if args.skip else None

    plot_flight(args.csv_file, include_cols=include, skip_cols=skip)


if __name__ == "__main__":
    main()
