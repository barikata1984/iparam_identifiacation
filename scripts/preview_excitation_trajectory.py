#!/usr/bin/env python3
"""
Preview an excitation trajectory in RViz.

Loads excitation_trajectory.json (metadata + trajectory [{t, q, dq, ddq}])
and publishes joint states to /preview/joint_states for RViz visualization.

Usage:
    1. Launch RViz with the robot model:
         roslaunch iparam_identification preview_trajectory.launch
    2. Run this script:
         rosrun iparam_identification preview_excitation_trajectory.py [--speed 0.2]
    3. Press Enter to start playback, 'r' to replay, 'q' to quit.
"""

import argparse
import json
import os
import sys
import threading

import numpy as np
import rospy
from sensor_msgs.msg import JointState

UR5E_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

DEFAULT_TRAJECTORY = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "data",
    "trajectories",
    "excitation_trajectory.json",
)


class ExcitationTrajectoryPreviewNode:
    """Preview excitation trajectory in RViz."""

    def __init__(self, trajectory_path: str, speed: float = 1.0):
        rospy.init_node("excitation_trajectory_preview", anonymous=True)

        with open(trajectory_path, "r") as f:
            data = json.load(f)

        metadata = data["metadata"]
        trajectory = data["trajectory"]

        self.times = np.array([step["t"] for step in trajectory])
        self.positions = np.array([step["q"] for step in trajectory])
        self.velocities = np.array([step["dq"] for step in trajectory])

        self.speed = speed
        self.duration = metadata["duration"]
        n_steps = len(trajectory)

        self.pub = rospy.Publisher("/preview/joint_states", JointState, queue_size=10)
        self.is_playing = False

        # Print trajectory info
        q_min_deg = np.rad2deg(self.positions.min(axis=0))
        q_max_deg = np.rad2deg(self.positions.max(axis=0))
        q0_deg = np.rad2deg(self.positions[0])
        dq_max = np.abs(self.velocities).max(axis=0)
        print(f"Loaded: {trajectory_path}")
        print(f"  Duration: {self.duration}s, Steps: {n_steps}, dt: {metadata['dt']}s")
        print(f"  Playback speed: {self.speed}x (real time: {self.duration / self.speed:.1f}s)")
        print(f"  Condition number: {metadata.get('condition_number', 'N/A')}")
        print(f"  q0 (deg): {q0_deg.round(1).tolist()}")
        print(f"  q_min (deg): {q_min_deg.round(1).tolist()}")
        print(f"  q_max (deg): {q_max_deg.round(1).tolist()}")
        print(f"  |dq|_max (rad/s): {dq_max.round(2).tolist()}")

    def publish_joint_state(self, index: int):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = UR5E_JOINT_NAMES
        msg.position = self.positions[index].tolist()
        msg.velocity = self.velocities[index].tolist()
        msg.effort = [0.0] * 6
        self.pub.publish(msg)

    def playback(self):
        self.is_playing = True
        current_index = 0
        playback_duration = self.duration / self.speed

        print(f"Playing trajectory (speed={self.speed}x, duration={playback_duration:.1f}s)...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown() and current_index < len(self.times):
            elapsed = (rospy.Time.now() - start_time).to_sec()
            # Map wall-clock elapsed to trajectory time
            t_traj = elapsed * self.speed

            while current_index < len(self.times) - 1 and self.times[current_index] < t_traj:
                current_index += 1

            self.publish_joint_state(current_index)

            progress = current_index / (len(self.times) - 1) * 100
            t = self.times[current_index]
            q_deg = np.rad2deg(self.positions[current_index])
            sys.stdout.write(
                f"\r  t={t:5.2f}/{self.duration:.2f}s"
                f" | q1={q_deg[0]:7.1f} q2={q_deg[1]:7.1f} q3={q_deg[2]:7.1f}"
                f" | {progress:5.1f}%"
            )
            sys.stdout.flush()
            rate.sleep()

        print("\n  Playback complete!")
        self.is_playing = False

    def interactive_mode(self):
        print("\n" + "=" * 60)
        print("Excitation Trajectory Preview")
        print("=" * 60)
        print("Commands:")
        print("  [Enter] - Play trajectory")
        print("  [s]     - Show start position")
        print("  [e]     - Show end position")
        print("  [r]     - Replay")
        print("  [1-9]   - Set speed (1=0.1x, 5=0.5x, 9=0.9x, 0=1.0x)")
        print("  [q]     - Quit")
        print("=" * 60)

        hold_index = 0

        def hold_loop():
            r = rospy.Rate(30)
            while not rospy.is_shutdown() and not self.is_playing:
                self.publish_joint_state(hold_index)
                r.sleep()

        hold_thread: threading.Thread | None = None

        while not rospy.is_shutdown():
            if hold_thread is None or not hold_thread.is_alive():
                if not self.is_playing:
                    hold_thread = threading.Thread(target=hold_loop, daemon=True)
                    hold_thread.start()
            try:
                cmd = input("\nCommand: ").strip().lower()
            except EOFError:
                break

            if cmd in ("", "p"):
                self.playback()
                hold_index = len(self.times) - 1
            elif cmd == "s":
                print("Showing start position...")
                hold_index = 0
                self.publish_joint_state(0)
            elif cmd == "e":
                print("Showing end position...")
                hold_index = len(self.times) - 1
                self.publish_joint_state(hold_index)
            elif cmd == "r":
                self.playback()
                hold_index = len(self.times) - 1
            elif cmd == "0":
                self.speed = 1.0
                print(f"Speed: {self.speed}x")
            elif cmd in [str(i) for i in range(1, 10)]:
                self.speed = int(cmd) / 10.0
                print(f"Speed: {self.speed}x")
            elif cmd == "q":
                print("Exiting...")
                break
            else:
                print(f"Unknown command: {cmd}")


def main():
    parser = argparse.ArgumentParser(description="Preview excitation trajectory in RViz")
    parser.add_argument("trajectory", nargs="?", default=DEFAULT_TRAJECTORY, help="Path to trajectory JSON")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (default: 1.0)")
    # rospy passes extra args, so parse only known
    args, _ = parser.parse_known_args()

    try:
        node = ExcitationTrajectoryPreviewNode(args.trajectory, speed=args.speed)
        node.interactive_mode()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
