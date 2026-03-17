#!/usr/bin/env python3
"""
Calibrate F/T sensor preload using 6-face cube poses.

Moves the UR5e to 6 orientations (flange Z in each cardinal direction),
samples ft_raw_wrench at each pose, and verifies that the preload is
orientation-invariant. Saves the preload constant to JSON.

Prerequisites:
    - Robot powered on and brake released
    - Gripper / payload REMOVED (bare flange)
    - No ROS driver needed (uses ur_rtde directly)

Usage:
    python3 calibrate_ft_preload.py <robot_ip>
    python3 calibrate_ft_preload.py 192.168.1.102
"""

from __future__ import annotations

import datetime
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

# Path setup
IPARAM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(IPARAM_ROOT / "src"))

from calibration.cube_poses import CubePose, compute_cube_poses  # noqa: E402

# ur_rtde imports (installed via pip)
import rtde_control  # noqa: E402
import rtde_receive  # noqa: E402

# Safety parameters
MOVE_SPEED = 0.5  # rad/s
MOVE_ACCELERATION = 0.3  # rad/s²
SETTLE_TIME = 2.0  # seconds to wait after arrival (vibration dampening)
SAMPLE_DURATION = 3.0  # seconds of sampling per pose
SAMPLE_RATE = 500  # Hz (RTDE default)


def sample_ft(
    rtde_r: rtde_receive.RTDEReceiveInterface,
    duration: float = SAMPLE_DURATION,
    rate: float = SAMPLE_RATE,
) -> tuple[np.ndarray, np.ndarray]:
    """Sample ft_raw_wrench and actual_TCP_force simultaneously.

    Returns:
        (raw_samples, tcp_samples): each (N, 6) array.
    """
    dt = 1.0 / rate
    n_samples = int(duration * rate)
    raw_samples = np.empty((n_samples, 6))
    tcp_samples = np.empty((n_samples, 6))
    for i in range(n_samples):
        raw_samples[i] = rtde_r.getFtRawWrench()
        tcp_samples[i] = rtde_r.getActualTCPForce()
        time.sleep(dt)
    return raw_samples, tcp_samples


def run_calibration(robot_ip: str) -> None:
    # Compute poses
    print("Computing 6-face cube poses (Pinocchio IK)...")
    poses = compute_cube_poses()
    print(f"  {len(poses)} poses computed successfully.\n")

    # Display all poses
    print("Planned poses:")
    print(f"  {'#':>2s}  {'Face':>4s}  {'Joint angles (deg)':60s}")
    print("  " + "-" * 70)
    for i, p in enumerate(poses):
        deg_str = np.array2string(p.joint_angles_deg, precision=1, separator=", ")
        print(f"  {i + 1:2d}  {p.label:>4s}  {deg_str}")
    print()

    # Safety warning
    print("=" * 60)
    print("  WARNING: ENSURE GRIPPER / PAYLOAD IS REMOVED")
    print("  The robot will move through 6 orientations.")
    print("  Ensure the workspace is clear.")
    print("=" * 60)
    input("  Press ENTER to connect to robot (Ctrl+C to abort)...")

    # Connect
    print(f"\nConnecting to {robot_ip}...")
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)
    print("Connected.\n")

    # Collect data for each pose
    results: list[dict] = []
    for i, pose in enumerate(poses):
        print(f"\n--- Pose {i + 1}/{len(poses)}: flange Z = {pose.label} ---")
        deg_str = np.array2string(pose.joint_angles_deg, precision=1, separator=", ")
        print(f"  Target (deg): {deg_str}")
        input(f"  Press ENTER to move to pose {i + 1} (Ctrl+C to abort)...")

        # Move
        print(f"  Moving (speed={MOVE_SPEED} rad/s)...")
        rtde_c.moveJ(pose.joint_angles_rad.tolist(), MOVE_SPEED, MOVE_ACCELERATION)

        # Settle
        print(f"  Settling ({SETTLE_TIME}s)...")
        time.sleep(SETTLE_TIME)

        # Sample
        print(
            f"  Sampling ft_raw_wrench + actual_TCP_force ({SAMPLE_DURATION}s @ {SAMPLE_RATE}Hz)..."
        )
        raw_samples, tcp_samples = sample_ft(rtde_r, SAMPLE_DURATION, SAMPLE_RATE)

        raw_mean = np.mean(raw_samples, axis=0)
        raw_std = np.std(raw_samples, axis=0)
        tcp_mean = np.mean(tcp_samples, axis=0)
        tcp_std = np.std(tcp_samples, axis=0)

        print(f"  ft_raw_wrench:")
        print(f"    Mean: {np.array2string(raw_mean, precision=3, separator=', ')}")
        print(f"    Std:  {np.array2string(raw_std, precision=4, separator=', ')}")
        print(f"  actual_TCP_force:")
        print(f"    Mean: {np.array2string(tcp_mean, precision=3, separator=', ')}")
        print(f"    Std:  {np.array2string(tcp_std, precision=4, separator=', ')}")

        results.append(
            {
                "pose_index": i,
                "label": pose.label,
                "flange_z": pose.flange_z_direction.tolist(),
                "joint_angles_rad": pose.joint_angles_rad.tolist(),
                "joint_angles_deg": pose.joint_angles_deg.tolist(),
                "n_samples": len(raw_samples),
                "mean": raw_mean.tolist(),
                "std": raw_std.tolist(),
                "tcp_force_mean": tcp_mean.tolist(),
                "tcp_force_std": tcp_std.tolist(),
            }
        )

    # Disconnect
    rtde_c.stopScript()
    rtde_r.disconnect()

    # Compute summary statistics — ft_raw_wrench
    all_raw_means = np.array([r["mean"] for r in results])  # (6, 6)
    all_raw_stds = np.array([r["std"] for r in results])

    preload_estimate = np.mean(all_raw_means, axis=0)
    inter_pose_std = np.std(all_raw_means, axis=0)
    mean_noise_std = np.mean(all_raw_stds, axis=0)
    max_deviation = np.max(np.abs(all_raw_means - preload_estimate), axis=0)

    # Compute summary statistics — actual_TCP_force
    all_tcp_means = np.array([r["tcp_force_mean"] for r in results])
    all_tcp_stds = np.array([r["tcp_force_std"] for r in results])

    tcp_mean_all = np.mean(all_tcp_means, axis=0)
    tcp_inter_pose_std = np.std(all_tcp_means, axis=0)
    tcp_mean_noise_std = np.mean(all_tcp_stds, axis=0)
    tcp_max_deviation = np.max(np.abs(all_tcp_means - tcp_mean_all), axis=0)

    # Display summary
    labels = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]

    print("\n" + "=" * 60)
    print("  ft_raw_wrench RESULTS")
    print("=" * 60)
    print(f"\n  {'':6s} {'Preload':>12s} {'Inter-pose σ':>14s} {'Max dev':>12s} {'Noise σ':>12s}")
    print("  " + "-" * 58)
    for j in range(6):
        print(
            f"  {labels[j]:6s} {preload_estimate[j]:12.3f} {inter_pose_std[j]:14.4f} "
            f"{max_deviation[j]:12.4f} {mean_noise_std[j]:12.4f}"
        )

    print("\n" + "=" * 60)
    print("  actual_TCP_force RESULTS")
    print("=" * 60)
    print(f"\n  {'':6s} {'Mean':>12s} {'Inter-pose σ':>14s} {'Max dev':>12s} {'Noise σ':>12s}")
    print("  " + "-" * 58)
    for j in range(6):
        print(
            f"  {labels[j]:6s} {tcp_mean_all[j]:12.3f} {tcp_inter_pose_std[j]:14.4f} "
            f"{tcp_max_deviation[j]:12.4f} {tcp_mean_noise_std[j]:12.4f}"
        )

    # Save results
    timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    results_dir = IPARAM_ROOT / "results"
    results_dir.mkdir(exist_ok=True)
    output_path = results_dir / f"ft_preload_calibration_{timestamp_str}.json"

    output_data = {
        "meta": {
            "timestamp": timestamp_str,
            "robot_ip": robot_ip,
            "n_poses": len(results),
            "sample_duration_s": SAMPLE_DURATION,
            "sample_rate_hz": SAMPLE_RATE,
            "settle_time_s": SETTLE_TIME,
        },
        "ft_raw_wrench": {
            "preload_estimate": preload_estimate.tolist(),
            "inter_pose_std": inter_pose_std.tolist(),
            "max_deviation": max_deviation.tolist(),
            "mean_noise_std": mean_noise_std.tolist(),
        },
        "actual_TCP_force": {
            "mean": tcp_mean_all.tolist(),
            "inter_pose_std": tcp_inter_pose_std.tolist(),
            "max_deviation": tcp_max_deviation.tolist(),
            "mean_noise_std": tcp_mean_noise_std.tolist(),
        },
        "poses": results,
    }

    with open(output_path, "w") as f:
        json.dump(output_data, f, indent=2)

    print(f"\n  Results saved to: {output_path}")
    print("=" * 60)


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python3 calibrate_ft_preload.py <robot_ip>")
        print("Example: python3 calibrate_ft_preload.py 192.168.1.102")
        sys.exit(1)

    robot_ip = sys.argv[1]

    try:
        run_calibration(robot_ip)
    except KeyboardInterrupt:
        print("\nAborted by user.")
        sys.exit(1)


if __name__ == "__main__":
    main()
