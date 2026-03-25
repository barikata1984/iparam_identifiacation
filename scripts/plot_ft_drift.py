#!/usr/bin/env python3
"""Plot F/T sensor drift from monitor_ft.py JSONL output.

Usage:
    python3 plot_ft_drift.py results/ft_monitor_2026-03-18_01-50-48.jsonl
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


LABELS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
FORCE_LABELS = ["Fx", "Fy", "Fz"]
TORQUE_LABELS = ["Tx", "Ty", "Tz"]


def load_jsonl(path: Path) -> list[dict]:
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python3 plot_ft_drift.py <jsonl_file>")
        sys.exit(1)

    path = Path(sys.argv[1])
    records = load_jsonl(path)
    n = len(records)
    if n < 2:
        print(f"Need at least 2 records, got {n}")
        sys.exit(1)

    # Extract time relative to first measurement
    t0 = records[0]["epoch"]
    t = np.array([r["epoch"] - t0 for r in records])

    # Extract raw wrench and compute drift from initial value
    raw = {label: np.array([r["ft_raw_wrench"][label] for r in records]) for label in LABELS}
    drift = {label: raw[label] - raw[label][0] for label in LABELS}

    # Also extract actual_TCP_force drift
    tcp = {label: np.array([r["actual_TCP_force"][label] for r in records]) for label in LABELS}
    tcp_drift = {label: tcp[label] - tcp[label][0] for label in LABELS}

    # --- Plot ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    fig.suptitle(
        f"F/T Sensor Drift  ({path.name}, {n} samples, {t[-1]:.0f}s)",
        fontsize=13,
        fontweight="bold",
    )

    colors = {
        "Fx": "#e74c3c",
        "Fy": "#2ecc71",
        "Fz": "#3498db",
        "Tx": "#e74c3c",
        "Ty": "#2ecc71",
        "Tz": "#3498db",
    }

    # Top-left: ft_raw_wrench force drift
    ax = axes[0, 0]
    for label in FORCE_LABELS:
        ax.plot(t, drift[label], "o-", color=colors[label], markersize=3, label=label)
    ax.set_ylabel("Drift [N]")
    ax.set_title("ft_raw_wrench — Force")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    # Top-right: ft_raw_wrench torque drift
    ax = axes[0, 1]
    for label in TORQUE_LABELS:
        ax.plot(t, drift[label], "o-", color=colors[label], markersize=3, label=label)
    ax.set_ylabel("Drift [Nm]")
    ax.set_title("ft_raw_wrench — Torque")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    # Bottom-left: actual_TCP_force force drift
    ax = axes[1, 0]
    for label in FORCE_LABELS:
        ax.plot(t, tcp_drift[label], "o-", color=colors[label], markersize=3, label=label)
    ax.set_ylabel("Drift [N]")
    ax.set_xlabel("Time [s]")
    ax.set_title("actual_TCP_force — Force")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    # Bottom-right: actual_TCP_force torque drift
    ax = axes[1, 1]
    for label in TORQUE_LABELS:
        ax.plot(t, tcp_drift[label], "o-", color=colors[label], markersize=3, label=label)
    ax.set_ylabel("Drift [Nm]")
    ax.set_xlabel("Time [s]")
    ax.set_title("actual_TCP_force — Torque")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="gray", linewidth=0.5)

    plt.tight_layout()

    # Save
    out_path = path.with_suffix(".png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
