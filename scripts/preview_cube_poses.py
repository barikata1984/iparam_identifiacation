#!/usr/bin/env python3
"""
Preview 6-face cube poses in RViz.

Publishes JointState messages to /preview/joint_states to visualize the
6 calibration poses used for ft_raw_wrench preload calibration.

Usage:
    1. Launch RViz with the robot model:
         roslaunch iparam_identification preview_trajectory.launch
    2. Run this script:
         rosrun iparam_identification preview_cube_poses.py
    3. Press Enter to step through poses, 's' to simulate full sequence, 'q' to quit.
"""

import argparse
import sys
import threading
from pathlib import Path

import numpy as np
import rospy
from sensor_msgs.msg import JointState

# Path setup
IPARAM_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(IPARAM_ROOT / "src"))

from calibration.cube_poses import compute_cube_poses  # noqa: E402
from calibration.septic_spline import septic_position, septic_velocity  # noqa: E402
from utilities.tool0_kinematics import JOINT_ORDER  # noqa: E402

# Dummy ft_raw_wrench preload values (typical for UR5e with tool)
_FT_PRELOAD_MEAN = np.array([24800.0, -20.0, 40.0, -0.1, 0.1, -0.05])
_FT_PRELOAD_SIGMA = np.array([5.0, 5.0, 5.0, 0.01, 0.01, 0.01])


class CubePosePreviewNode:
    """Preview 6-face cube poses in RViz."""

    def __init__(self, hold_time: float = 3.0, transition_time: float = 3.0):
        rospy.init_node("cube_pose_preview", anonymous=True)
        self.pub = rospy.Publisher("/preview/joint_states", JointState, queue_size=10)

        self.hold_time = hold_time
        self.transition_time = transition_time

        print("Computing 6-face cube poses...")
        self.poses = compute_cube_poses()
        print(f"  {len(self.poses)} poses ready.\n")

        # Print pose summary
        print(f"  {'#':>2s}  {'Face':>4s}  {'Joint angles (deg)':60s}")
        print("  " + "-" * 70)
        for i, p in enumerate(self.poses):
            deg_str = np.array2string(p.joint_angles_deg, precision=1, separator=", ")
            print(f"  {i + 1:2d}  {p.label:>4s}  {deg_str}")

        self.current_index = 0
        self._hold_active = True
        self._simulating = False

        # Background thread to keep publishing current pose (RViz needs continuous updates)
        self._hold_thread = threading.Thread(target=self._hold_loop, daemon=True)
        self._hold_thread.start()

    def _publish_joint_state(
        self, position: np.ndarray, velocity: np.ndarray | None = None
    ) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(JOINT_ORDER)
        msg.position = position.tolist()
        msg.velocity = (velocity if velocity is not None else np.zeros(6)).tolist()
        msg.effort = [0.0] * 6
        self.pub.publish(msg)

    def _publish_pose(self, index: int) -> None:
        self._publish_joint_state(self.poses[index].joint_angles_rad)

    def _hold_loop(self) -> None:
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and self._hold_active:
            if not self._simulating:
                self._publish_pose(self.current_index)
            rate.sleep()

    def interactive_mode(self) -> None:
        print("\n" + "=" * 60)
        print("  Cube Pose Preview")
        print("=" * 60)
        print("  Commands:")
        print("    [Enter] - Next pose")
        print("    [p]     - Previous pose")
        print("    [1-6]   - Jump to pose #")
        print("    [a]     - Auto-cycle all poses (3s each)")
        print("    [s]     - Simulate full transition sequence")
        print("    [q]     - Quit")
        print("=" * 60)

        self._show_current()

        while not rospy.is_shutdown():
            try:
                cmd = input("\nCommand: ").strip().lower()
            except EOFError:
                break

            if cmd == "q":
                print("Exiting...")
                break
            elif cmd in ("", "n"):
                self.current_index = (self.current_index + 1) % len(self.poses)
                self._show_current()
            elif cmd == "p":
                self.current_index = (self.current_index - 1) % len(self.poses)
                self._show_current()
            elif cmd in [str(i) for i in range(1, 7)]:
                self.current_index = int(cmd) - 1
                self._show_current()
            elif cmd == "a":
                self._auto_cycle()
            elif cmd == "s":
                self._simulate_sequence()
            else:
                print(f"  Unknown command: {cmd}")

        self._hold_active = False

    def _show_current(self) -> None:
        pose = self.poses[self.current_index]
        deg_str = np.array2string(pose.joint_angles_deg, precision=1, separator=", ")
        print(f"  Pose {self.current_index + 1}/{len(self.poses)}: flange Z = {pose.label}")
        print(f"    q (deg): {deg_str}")

    def _auto_cycle(self) -> None:
        print("  Auto-cycling all poses (3s each)...")
        for i in range(len(self.poses)):
            if rospy.is_shutdown():
                break
            self.current_index = i
            self._show_current()
            rospy.sleep(3.0)
        print("  Auto-cycle complete.")

    def _simulate_sequence(self) -> None:
        """Simulate full transition sequence with septic spline interpolation."""
        n_poses = len(self.poses)
        T_trans = self.transition_time
        T_hold = self.hold_time
        total = (n_poses - 1) * T_trans + n_poses * T_hold
        print(
            f"\n  Simulating {n_poses} poses "
            f"(transition={T_trans:.1f}s, hold={T_hold:.1f}s, total={total:.1f}s)"
        )
        print("  " + "-" * 60)

        self._simulating = True
        rate = rospy.Rate(100)

        try:
            for i in range(n_poses):
                if rospy.is_shutdown():
                    break

                # --- Transition from previous pose ---
                if i > 0:
                    q0 = self.poses[i - 1].joint_angles_rad
                    q1 = self.poses[i].joint_angles_rad
                    start = rospy.Time.now()

                    while not rospy.is_shutdown():
                        t = (rospy.Time.now() - start).to_sec()
                        if t >= T_trans:
                            break
                        q = septic_position(q0, q1, t, T_trans)
                        dq = septic_velocity(q0, q1, t, T_trans)
                        self._publish_joint_state(q, dq)

                        # Progress bar
                        frac = t / T_trans
                        bar_w = 20
                        filled = int(frac * bar_w)
                        bar = "=" * filled + ">" + " " * (bar_w - filled - 1)
                        q_deg = np.rad2deg(q)
                        sys.stdout.write(
                            f"\r  Transition {i}→{i + 1} [{bar}] "
                            f"{t:.1f}/{T_trans:.1f}s  "
                            f"q1={q_deg[0]:6.1f} q2={q_deg[1]:6.1f} q3={q_deg[2]:6.1f}"
                        )
                        sys.stdout.flush()
                        rate.sleep()

                    # Publish final pose exactly
                    self._publish_joint_state(q1)
                    sys.stdout.write("\r" + " " * 80 + "\r")
                    sys.stdout.flush()
                    print(f"  Transition {i}→{i + 1} complete.")

                # --- Update current index for hold thread ---
                self.current_index = i
                pose = self.poses[i]

                # --- Hold phase ---
                print(f"\n  Pose {i + 1}/{n_poses} ({pose.label}): holding {T_hold:.1f}s...")
                start = rospy.Time.now()
                while not rospy.is_shutdown():
                    t = (rospy.Time.now() - start).to_sec()
                    if t >= T_hold:
                        break
                    self._publish_joint_state(pose.joint_angles_rad)
                    rate.sleep()

                # --- Measurement simulation ---
                self._simulate_measurement(i, n_poses, pose.label)

        finally:
            self._simulating = False

        print("\n  " + "=" * 60)
        print("  Simulation complete. Returning to interactive mode.")

    def _simulate_measurement(self, idx: int, n_poses: int, label: str) -> None:
        """Simulate ft_raw_wrench sampling with dummy noisy data."""
        T_sample = self.hold_time
        n_samples = int(T_sample / 0.5)
        samples = np.zeros((n_samples, 6))
        rng = np.random.default_rng()

        rate = rospy.Rate(100)
        start = rospy.Time.now()
        sample_idx = 0

        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start).to_sec()
            if t >= T_sample:
                break

            # Publish current pose to keep RViz alive
            self._publish_joint_state(self.poses[idx].joint_angles_rad)

            # Generate and display sample at 0.5s intervals
            new_sample_idx = min(int(t / 0.5), n_samples - 1)
            if new_sample_idx > sample_idx or (sample_idx == 0 and t < 0.5):
                if new_sample_idx > sample_idx:
                    sample_idx = new_sample_idx
                if sample_idx < n_samples:
                    samples[sample_idx] = _FT_PRELOAD_MEAN + rng.normal(scale=_FT_PRELOAD_SIGMA)

            # Progress bar
            frac = t / T_sample
            bar_w = 20
            filled = int(frac * bar_w)
            bar = "=" * filled + ">" + " " * max(0, bar_w - filled - 1)
            sys.stdout.write(
                f"\r  Pose {idx + 1}/{n_poses} ({label}): "
                f"Sampling ft_raw_wrench [{bar}] {t:.1f}/{T_sample:.1f}s"
            )
            sys.stdout.flush()
            rate.sleep()

        # Generate any remaining samples
        for j in range(n_samples):
            if np.all(samples[j] == 0):
                samples[j] = _FT_PRELOAD_MEAN + rng.normal(scale=_FT_PRELOAD_SIGMA)

        sys.stdout.write("\r" + " " * 80 + "\r")
        sys.stdout.flush()

        mean = samples.mean(axis=0)
        std = samples.std(axis=0)
        mean_str = ", ".join(f"{v:9.1f}" for v in mean)
        std_str = ", ".join(f"{v:9.3f}" for v in std)
        print(f"  Pose {idx + 1}/{n_poses} ({label}): Sampling complete ({n_samples} samples)")
        print(f"    Mean: [{mean_str}]")
        print(f"    Std:  [{std_str}]")


def main():
    parser = argparse.ArgumentParser(description="Preview 6-face cube poses in RViz")
    parser.add_argument(
        "--hold-time",
        type=float,
        default=3.0,
        help="Hold time at each pose [s] (default: 3.0)",
    )
    parser.add_argument(
        "--transition-time",
        type=float,
        default=3.0,
        help="Transition time between poses [s] (default: 3.0)",
    )
    args, _ = parser.parse_known_args()

    try:
        node = CubePosePreviewNode(hold_time=args.hold_time, transition_time=args.transition_time)
        node.interactive_mode()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
