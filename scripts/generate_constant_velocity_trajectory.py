#!/usr/bin/env python3
"""
Generate constant velocity trajectory for specified joint.

Trajectory profile:
- Acceleration phase: 1°/s² for 5s (0-5s)
- Constant velocity phase: 5°/s for variable duration
- Deceleration phase: -1°/s² for 5s

Supports J1 and J3 constant velocity tests.
"""

import argparse

import json
import numpy as np


def generate_trapezoidal_trajectory(
    start_deg: float,
    end_deg: float,
    acc_deg_s2: float,
    acc_duration: float,
    control_freq: float,
) -> dict:
    """
    Generate a trapezoidal velocity profile trajectory.

    Parameters
    ----------
    start_deg : float
        Start position in degrees
    end_deg : float
        End position in degrees
    acc_deg_s2 : float
        Acceleration magnitude in deg/s²
    acc_duration : float
        Duration of acceleration/deceleration phase in seconds
    control_freq : float
        Control frequency in Hz

    Returns
    -------
    dict
        Trajectory data with positions, velocities, accelerations, and times
    """
    dt = 1.0 / control_freq
    total_distance = end_deg - start_deg
    direction = np.sign(total_distance)
    distance = abs(total_distance)

    # Acceleration phase
    acc = acc_deg_s2
    t_acc = acc_duration
    v_max = acc * t_acc  # Maximum velocity achieved
    d_acc = 0.5 * acc * t_acc**2  # Distance during acceleration

    # Deceleration phase (symmetric)
    t_dec = acc_duration
    d_dec = 0.5 * acc * t_dec**2  # Distance during deceleration

    # Constant velocity phase
    d_const = distance - d_acc - d_dec
    if d_const < 0:
        raise ValueError(
            f"Distance {distance}° is too short for the given acceleration profile. "
            f"Minimum distance: {d_acc + d_dec}°"
        )
    t_const = d_const / v_max

    # Total time
    t_total = t_acc + t_const + t_dec

    print(f"Trajectory Parameters:")
    print(f"  Start: {start_deg}°, End: {end_deg}°")
    print(f"  Total distance: {distance}°")
    print(f"  Acceleration: {acc}°/s² for {t_acc}s → v_max = {v_max}°/s")
    print(f"  Constant velocity: {v_max}°/s for {t_const:.3f}s")
    print(f"  Deceleration: -{acc}°/s² for {t_dec}s")
    print(f"  Total time: {t_total:.3f}s")
    print(f"  Control frequency: {control_freq} Hz")
    print(f"  dt: {dt:.6f}s")

    # Generate time array
    n_samples = int(np.ceil(t_total * control_freq)) + 1
    times = np.arange(n_samples) * dt

    positions = []
    velocities = []
    accelerations = []

    for t in times:
        if t <= t_acc:
            # Acceleration phase
            a = acc
            v = acc * t
            p = start_deg + direction * 0.5 * acc * t**2
        elif t <= t_acc + t_const:
            # Constant velocity phase
            t_in_phase = t - t_acc
            a = 0.0
            v = v_max
            p = start_deg + direction * (d_acc + v_max * t_in_phase)
        else:
            # Deceleration phase
            t_in_phase = t - t_acc - t_const
            a = -acc
            v = v_max - acc * t_in_phase
            p = start_deg + direction * (d_acc + d_const + v_max * t_in_phase - 0.5 * acc * t_in_phase**2)

        positions.append(p)
        velocities.append(direction * v)
        accelerations.append(direction * a)

    # Ensure final position is exact
    positions[-1] = end_deg
    velocities[-1] = 0.0
    accelerations[-1] = 0.0

    print(f"  Number of samples: {n_samples}")
    print(f"  Actual duration: {times[-1]:.3f}s")

    return {
        "times": times.tolist(),
        "positions_deg": positions,
        "velocities_deg_s": velocities,
        "accelerations_deg_s2": accelerations,
        "n_samples": n_samples,
        "dt": dt,
        "t_total": t_total,
        "phases": {
            "acceleration": {"start": 0.0, "end": t_acc, "duration": t_acc},
            "constant_velocity": {"start": t_acc, "end": t_acc + t_const, "duration": t_const},
            "deceleration": {"start": t_acc + t_const, "end": t_total, "duration": t_dec},
        },
        "v_max_deg_s": v_max,
    }


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def generate_single_joint_trajectory(
    joint_index: int,
    start_position_deg: list,
    end_position_deg: list,
    acc_deg_s2: float = 1.0,
    acc_duration: float = 5.0,
    control_freq: float = 30.0,
    description: str = "",
) -> dict:
    """
    Generate constant velocity test trajectory for a single joint.

    Parameters
    ----------
    joint_index : int
        Index of the joint to move (0=J1, 1=J2, ..., 5=J6)
    start_position_deg : list
        Start position for all 6 joints in degrees
    end_position_deg : list
        End position for all 6 joints in degrees
    acc_deg_s2 : float
        Acceleration magnitude in deg/s²
    acc_duration : float
        Duration of acceleration/deceleration phase in seconds
    control_freq : float
        Control frequency in Hz
    description : str
        Description of the trajectory

    Returns
    -------
    dict
        Full trajectory data
    """
    # Generate trajectory for the moving joint
    moving_traj = generate_trapezoidal_trajectory(
        start_deg=start_position_deg[joint_index],
        end_deg=end_position_deg[joint_index],
        acc_deg_s2=acc_deg_s2,
        acc_duration=acc_duration,
        control_freq=control_freq,
    )

    full_trajectory = {
        "description": description,
        "joint_names": JOINT_NAMES,
        "moving_joint_index": joint_index,
        "moving_joint_name": JOINT_NAMES[joint_index],
        "start_position_deg": start_position_deg,
        "end_position_deg": end_position_deg,
        "control_freq_hz": control_freq,
        "moving_joint_trajectory": moving_traj,
        "trajectory": {
            "times": moving_traj["times"],
            "positions_deg": [],
            "velocities_deg_s": [],
            "accelerations_deg_s2": [],
        },
    }

    # Build full 6-joint trajectory
    for i in range(moving_traj["n_samples"]):
        pos = start_position_deg.copy()
        vel = [0.0] * 6
        acc = [0.0] * 6

        pos[joint_index] = moving_traj["positions_deg"][i]
        vel[joint_index] = moving_traj["velocities_deg_s"][i]
        acc[joint_index] = moving_traj["accelerations_deg_s2"][i]

        full_trajectory["trajectory"]["positions_deg"].append(pos)
        full_trajectory["trajectory"]["velocities_deg_s"].append(vel)
        full_trajectory["trajectory"]["accelerations_deg_s2"].append(acc)

    return full_trajectory


def generate_j1_trajectory():
    """Generate J1 constant velocity test trajectory."""
    return generate_single_joint_trajectory(
        joint_index=0,
        start_position_deg=[60.0, -90.0, 90.0, -90.0, -90.0, 90.0],
        end_position_deg=[120.0, -90.0, 90.0, -90.0, -90.0, 90.0],
        acc_deg_s2=1.0,
        acc_duration=5.0,
        control_freq=30.0,
        description="J1 constant velocity test trajectory (60° → 120°)",
    )


def generate_j3_trajectory():
    """Generate J3 constant velocity test trajectory."""
    return generate_single_joint_trajectory(
        joint_index=2,  # elbow_joint
        start_position_deg=[90.0, -90.0, 120.0, -90.0, -90.0, 90.0],
        end_position_deg=[90.0, -90.0, 30.0, -90.0, -90.0, 90.0],
        acc_deg_s2=1.0,
        acc_duration=5.0,
        control_freq=30.0,
        description="J3 constant velocity test trajectory (120° → 30°)",
    )


def main():
    parser = argparse.ArgumentParser(description="Generate constant velocity trajectory")
    parser.add_argument(
        "--joint", "-j",
        type=int,
        choices=[1, 3],
        default=1,
        help="Joint to generate trajectory for (1=J1, 3=J3)",
    )
    args = parser.parse_args()

    if args.joint == 1:
        trajectory = generate_j1_trajectory()
        output_path = "/root/osx-ur/catkin_ws/src/iparam_identification/results/j1_constant_velocity_trajectory.json"
    elif args.joint == 3:
        trajectory = generate_j3_trajectory()
        output_path = "/root/osx-ur/catkin_ws/src/iparam_identification/results/j3_constant_velocity_trajectory.json"

    with open(output_path, "w") as f:
        json.dump(trajectory, f, indent=2)

    print(f"\nTrajectory saved to: {output_path}")

    # Print summary
    phases = trajectory["moving_joint_trajectory"]["phases"]
    print(f"\nPhase Summary:")
    print(f"  Acceleration:      {phases['acceleration']['start']:.1f}s - {phases['acceleration']['end']:.1f}s")
    print(f"  Constant Velocity: {phases['constant_velocity']['start']:.1f}s - {phases['constant_velocity']['end']:.1f}s")
    print(f"  Deceleration:      {phases['deceleration']['start']:.1f}s - {phases['deceleration']['end']:.1f}s")


if __name__ == "__main__":
    main()
