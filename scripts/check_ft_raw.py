#!/usr/bin/env python3
"""
Quick check: read ft_raw_wrench from RTDE and compare with /wrench topic values.

If ft_raw_wrench shows non-zero values (gripper weight) while /wrench shows ~0,
then ft_raw_wrench is NOT affected by startup zeroing.

Usage:
    python3 check_ft_raw.py <robot_ip>
    python3 check_ft_raw.py 192.168.1.102
"""

import sys
import time

import rtde_receive


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_ft_raw.py <robot_ip>")
        sys.exit(1)

    robot_ip = sys.argv[1]
    print(f"Connecting to {robot_ip}...")

    rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
    print("Connected.\n")

    print("Reading 5 samples (1 Hz)...\n")
    print(f"{'':4s} {'--- actual_TCP_force (compensated) ---':>45s}   {'--- ft_raw_wrench (raw) ---':>35s}")
    print(f"{'#':4s} {'Fx':>8s} {'Fy':>8s} {'Fz':>8s} {'Tx':>8s} {'Ty':>8s} {'Tz':>8s}   "
          f"{'Fx':>8s} {'Fy':>8s} {'Fz':>8s} {'Tx':>8s} {'Ty':>8s} {'Tz':>8s}")
    print("-" * 110)

    for i in range(5):
        tcp_force = rtde.getActualTCPForce()
        raw_wrench = rtde.getFtRawWrench()

        tcp_str = " ".join(f"{v:8.3f}" for v in tcp_force)
        raw_str = " ".join(f"{v:8.3f}" for v in raw_wrench)
        print(f"{i:4d} {tcp_str}   {raw_str}")

        time.sleep(1.0)

    rtde.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    main()
