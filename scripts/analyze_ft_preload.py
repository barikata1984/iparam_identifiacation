#!/usr/bin/env python3
"""
Analyze ft_preload_calibration results by transforming raw wrench to base frame.

Loads a calibration JSON, computes FK for each pose to get the sensor-to-base
rotation, and transforms ft_raw_wrench from sensor frame to base frame.

Usage:
    python3 analyze_ft_preload.py <calibration_json>
    python3 analyze_ft_preload.py results/ft_preload_calibration_2026-03-16_18-42-36.json
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

# Path setup
IPARAM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(IPARAM_ROOT / "src"))

from utilities.tool0_kinematics import DEFAULT_URDF_PATH  # noqa: E402


def analyze(calibration_path: str) -> None:
    with open(calibration_path) as f:
        data = json.load(f)

    # Build Pinocchio model
    model = pin.buildModelFromUrdf(DEFAULT_URDF_PATH)
    pin_data = model.createData()
    tool0_id = model.getFrameId("tool0")

    poses = data["poses"]
    n_poses = len(poses)

    print(f"Loaded: {calibration_path}")
    print(f"  {n_poses} poses\n")

    # Transform each pose's raw wrench to base frame
    print("=" * 80)
    print("  Sensor-frame raw wrench (as measured)")
    print("=" * 80)
    print(
        f"  {'#':>2s}  {'Face':>4s}  {'Fx':>10s} {'Fy':>10s} {'Fz':>10s}"
        f"  {'Tx':>8s} {'Ty':>8s} {'Tz':>8s}"
    )
    print("  " + "-" * 74)

    f_base_all = np.empty((n_poses, 3))
    t_base_all = np.empty((n_poses, 3))
    f_sensor_all = np.empty((n_poses, 3))

    for i, pose in enumerate(poses):
        q = np.array(pose["joint_angles_rad"])
        mean = np.array(pose["mean"])
        f_sensor = mean[:3]
        t_sensor = mean[3:]
        f_sensor_all[i] = f_sensor

        print(
            f"  {i + 1:2d}  {pose['label']:>4s}"
            f"  {f_sensor[0]:10.2f} {f_sensor[1]:10.2f} {f_sensor[2]:10.2f}"
            f"  {t_sensor[0]:8.3f} {t_sensor[1]:8.3f} {t_sensor[2]:8.3f}"
        )

        # FK to get rotation matrix
        pin.forwardKinematics(model, pin_data, q)
        pin.updateFramePlacements(model, pin_data)
        R = pin_data.oMf[tool0_id].rotation.copy()

        # Transform to base frame: F_base = R @ F_sensor
        f_base_all[i] = R @ f_sensor
        t_base_all[i] = R @ t_sensor

    print()
    print("=" * 80)
    print("  Base-frame wrench (R @ raw)")
    print("=" * 80)
    print(
        f"  {'#':>2s}  {'Face':>4s}  {'Fx':>10s} {'Fy':>10s} {'Fz':>10s}"
        f"  {'Tx':>8s} {'Ty':>8s} {'Tz':>8s}"
    )
    print("  " + "-" * 74)

    for i, pose in enumerate(poses):
        f = f_base_all[i]
        t = t_base_all[i]
        print(
            f"  {i + 1:2d}  {pose['label']:>4s}"
            f"  {f[0]:10.2f} {f[1]:10.2f} {f[2]:10.2f}"
            f"  {t[0]:8.3f} {t[1]:8.3f} {t[2]:8.3f}"
        )

    # Statistics
    f_mean = f_base_all.mean(axis=0)
    f_std = f_base_all.std(axis=0)
    f_max_dev = np.max(np.abs(f_base_all - f_mean), axis=0)

    t_mean = t_base_all.mean(axis=0)
    t_std = t_base_all.std(axis=0)
    t_max_dev = np.max(np.abs(t_base_all - t_mean), axis=0)

    print()
    print("=" * 80)
    print("  Base-frame statistics")
    print("=" * 80)
    print(f"  {'':>10s}  {'Fx':>10s} {'Fy':>10s} {'Fz':>10s}  {'Tx':>8s} {'Ty':>8s} {'Tz':>8s}")
    print("  " + "-" * 64)
    print(
        f"  {'Mean':>10s}  {f_mean[0]:10.2f} {f_mean[1]:10.2f} {f_mean[2]:10.2f}"
        f"  {t_mean[0]:8.3f} {t_mean[1]:8.3f} {t_mean[2]:8.3f}"
    )
    print(
        f"  {'Std':>10s}  {f_std[0]:10.4f} {f_std[1]:10.4f} {f_std[2]:10.4f}"
        f"  {t_std[0]:8.4f} {t_std[1]:8.4f} {t_std[2]:8.4f}"
    )
    print(
        f"  {'Max dev':>10s}  {f_max_dev[0]:10.4f} {f_max_dev[1]:10.4f} {f_max_dev[2]:10.4f}"
        f"  {t_max_dev[0]:8.4f} {t_max_dev[1]:8.4f} {t_max_dev[2]:8.4f}"
    )

    # Gravity analysis
    print()
    print("=" * 80)
    print("  Gravity analysis")
    print("=" * 80)
    print(f"  Base Fz mean: {f_mean[2]:.2f}")
    print(f"  Base Fz std:  {f_std[2]:.4f}")
    print(f"  If Fz contains m*g, and assuming 1 raw = 1 N:")
    print(f"    Implied mass = Fz / 9.81 = {f_mean[2] / 9.81:.1f} kg")
    print(f"  Fx, Fy should be ~constant (no gravity contribution):")
    print(f"    Fx std: {f_std[0]:.4f},  Fy std: {f_std[1]:.4f}")


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_ft_preload.py <calibration_json>")
        sys.exit(1)
    analyze(sys.argv[1])


if __name__ == "__main__":
    main()
