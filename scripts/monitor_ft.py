#!/usr/bin/env python3
"""Continuously monitor F/T sensor readings via RTDE and log to JSON Lines.

Each measurement is appended to the output file immediately (one JSON object
per line), so data is never lost even on unexpected termination.

Usage:
    python3 monitor_ft.py 192.168.55.20
    python3 monitor_ft.py 192.168.55.20 --interval 30
    python3 monitor_ft.py 192.168.55.20 --interval 10 -o my_log.jsonl
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rtde_receive


LABELS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
UNITS = ["N", "N", "N", "Nm", "Nm", "Nm"]
DEFAULT_INTERVAL = 60.0  # seconds
DEFAULT_AVG_SAMPLES = 50
DEFAULT_AVG_RATE = 50.0  # Hz


def read_avg(
    rtde: rtde_receive.RTDEReceiveInterface,
    n: int = DEFAULT_AVG_SAMPLES,
    rate: float = DEFAULT_AVG_RATE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read n samples, return averages of (actual_TCP_force, ft_raw_wrench, joint_angles)."""
    dt = 1.0 / rate
    tcp_samples: list[list[float]] = []
    raw_samples: list[list[float]] = []
    q_samples: list[list[float]] = []
    for _ in range(n):
        tcp_samples.append(rtde.getActualTCPForce())
        raw_samples.append(rtde.getFtRawWrench())
        q_samples.append(rtde.getActualQ())
        time.sleep(dt)
    return (
        np.mean(tcp_samples, axis=0),
        np.mean(raw_samples, axis=0),
        np.mean(q_samples, axis=0),
    )


def build_record(
    seq: int,
    tcp: np.ndarray,
    raw: np.ndarray,
    q: np.ndarray,
) -> dict:
    """Build a single measurement record."""
    now = datetime.now(timezone.utc)
    return {
        "seq": seq,
        "timestamp_utc": now.isoformat(timespec="milliseconds"),
        "epoch": round(now.timestamp(), 3),
        "actual_TCP_force": {LABELS[i]: round(float(tcp[i]), 4) for i in range(6)},
        "ft_raw_wrench": {LABELS[i]: round(float(raw[i]), 4) for i in range(6)},
        "joint_angles_rad": [round(float(q[i]), 6) for i in range(6)],
    }


def append_jsonl(path: Path, record: dict) -> None:
    """Append one JSON line to file."""
    with open(path, "a") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def print_record(record: dict, prev_raw: dict | None) -> None:
    """Pretty-print a measurement to stdout."""
    seq = record["seq"]
    ts = record["timestamp_utc"]
    raw = record["ft_raw_wrench"]
    tcp = record["actual_TCP_force"]

    print(f"\n--- [{seq:>4d}] {ts} ---")
    header = f"  {'':4s} {'TCP_force':>12s} {'raw_wrench':>12s}"
    if prev_raw is not None:
        header += f" {'Δraw':>10s}"
    print(header)

    for label, unit in zip(LABELS, UNITS):
        line = f"  {label:4s} {tcp[label]:>11.4f}{unit:3s} {raw[label]:>12.4f}"
        if prev_raw is not None:
            delta = raw[label] - prev_raw[label]
            line += f" {delta:>+10.4f}"
        print(line)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Continuously monitor F/T sensor and log to JSON Lines.",
    )
    parser.add_argument("robot_ip", help="Robot IP address (e.g. 192.168.55.20)")
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL,
        help=f"Measurement interval in seconds (default: {DEFAULT_INTERVAL})",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output JSONL file path (default: results/ft_monitor_<timestamp>.jsonl)",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=DEFAULT_AVG_SAMPLES,
        help=f"Number of samples to average per measurement (default: {DEFAULT_AVG_SAMPLES})",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # Output path
    script_dir = Path(__file__).resolve().parent
    results_dir = script_dir.parent / "results"
    if args.output:
        out_path = Path(args.output)
    else:
        results_dir.mkdir(exist_ok=True)
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        out_path = results_dir / f"ft_monitor_{stamp}.jsonl"

    # Connect
    print(f"Connecting to {args.robot_ip}...")
    rtde = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    print(f"Connected.  interval={args.interval}s  samples={args.samples}  output={out_path}\n")

    # Graceful shutdown
    running = True

    def _shutdown(signum: int, frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    seq = 0
    prev_raw: dict | None = None

    print("Monitoring started. Press Ctrl+C to stop.")

    try:
        while running:
            tcp, raw, q = read_avg(rtde, n=args.samples)
            record = build_record(seq, tcp, raw, q)
            append_jsonl(out_path, record)
            print_record(record, prev_raw)
            prev_raw = record["ft_raw_wrench"]
            seq += 1

            # Sleep in small increments so Ctrl+C is responsive
            deadline = time.monotonic() + args.interval
            while running and time.monotonic() < deadline:
                time.sleep(min(0.5, deadline - time.monotonic()))
    finally:
        rtde.disconnect()
        print(f"\n\nStopped after {seq} measurements.  Output: {out_path}")


if __name__ == "__main__":
    main()
