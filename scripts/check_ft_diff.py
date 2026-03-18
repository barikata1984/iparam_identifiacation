#!/usr/bin/env python3
"""
Measure F/T difference before and after payload attachment via RTDE.

No zero_ftsensor is called. The difference of actual_TCP_force readings
between bare flange and payload-attached states should equal the payload
weight (-mg in tool Z when flange points up).

Usage:
    python3 check_ft_diff.py 192.168.55.20
"""

import sys

import numpy as np
import rtde_receive


def read_avg(rtde: rtde_receive.RTDEReceiveInterface, n: int = 50):
    """Read n samples of actual_TCP_force and ft_raw_wrench, return averages."""
    import time

    tcp_samples = []
    raw_samples = []
    for _ in range(n):
        tcp_samples.append(rtde.getActualTCPForce())
        raw_samples.append(rtde.getFtRawWrench())
        time.sleep(0.02)  # 50 Hz
    return np.mean(tcp_samples, axis=0), np.mean(raw_samples, axis=0)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_ft_diff.py <robot_ip>")
        sys.exit(1)

    robot_ip = sys.argv[1]
    print(f"Connecting to {robot_ip}...")
    rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
    print("Connected.\n")

    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    units = ["N", "N", "N", "Nm", "Nm", "Nm"]

    # Step 1: Read bare flange
    input("  [1] Ensure BARE FLANGE at mounting pose (flange UP). Press ENTER to measure...")
    bare_tcp, bare_raw = read_avg(rtde)
    print("  Bare flange (actual_TCP_force, 50-sample avg):")
    for i, (l, u) in enumerate(zip(labels, units)):
        print(f"    {l}: {bare_tcp[i]:>10.4f} {u}")
    print("  Bare flange (ft_raw_wrench, 50-sample avg):")
    for i, (l, u) in enumerate(zip(labels, units)):
        print(f"    {l}: {bare_raw[i]:>12.2f}")

    # Step 2: Attach payload
    print()
    input("  [2] ATTACH GRIPPER / PAYLOAD. Press ENTER to measure...")
    loaded_tcp, loaded_raw = read_avg(rtde)
    print("  With payload (actual_TCP_force, 50-sample avg):")
    for i, (l, u) in enumerate(zip(labels, units)):
        print(f"    {l}: {loaded_tcp[i]:>10.4f} {u}")
    print("  With payload (ft_raw_wrench, 50-sample avg):")
    for i, (l, u) in enumerate(zip(labels, units)):
        print(f"    {l}: {loaded_raw[i]:>12.2f}")

    # Step 3: Differences
    diff_tcp = loaded_tcp - bare_tcp
    diff_raw = loaded_raw - bare_raw
    print()
    print("  === Difference (payload - bare) ===")
    print(f"  {'':4s} {'actual_TCP_force':>16s}  {'ft_raw_wrench':>14s}")
    for i, (l, u) in enumerate(zip(labels, units)):
        print(f"    {l}: {diff_tcp[i]:>12.4f} {u:3s}  {diff_raw[i]:>14.2f}")

    fmag_tcp = np.linalg.norm(diff_tcp[:3])
    fmag_raw = np.linalg.norm(diff_raw[:3])
    m_tcp = fmag_tcp / 9.81
    print()
    print(f"  actual_TCP_force: |F_diff| = {fmag_tcp:.4f} N  =>  m = {m_tcp:.4f} kg")
    print(f"  ft_raw_wrench:   |F_diff| = {fmag_raw:.2f} (unknown units)")
    if abs(diff_raw[2]) > 0.1:
        ratio = diff_tcp[2] / diff_raw[2]
        print(f"  Fz ratio (TCP/raw): {ratio:.4f} N per raw unit")

    rtde.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    main()
