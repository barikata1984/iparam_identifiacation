#!/usr/bin/env python3
"""
Preview an optimized excitation trajectory in RViz.

Loads a trajectory JSON containing pre-computed frames [q, qdot, qddot] and
publishes joint states to /preview/joint_states for RViz visualization.

Usage:
    1. Launch RViz with the robot model:
         roslaunch iparam_identification preview_trajectory.launch
    2. Run this script:
         rosrun iparam_identification preview_optimized_trajectory.py
    3. Press Enter to start playback, 'r' to replay, 'q' to quit.
"""

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
    "optimized_trajectory_box_two_stage.json",
)


class OptimizedTrajectoryPreviewNode:
    """Preview optimized excitation trajectory in RViz."""

    def __init__(self, trajectory_path: str):
        rospy.init_node("optimized_trajectory_preview", anonymous=True)

        with open(trajectory_path, "r") as f:
            data = json.load(f)

        config = data["config"]
        frames = data["frames"]
        duration = config["duration"]
        fps = config["fps"]

        # frames[i] = [q, qdot, qddot], each (6,)
        self.positions = np.array([f[0] for f in frames])
        self.velocities = np.array([f[1] for f in frames])
        n_steps = len(frames)
        self.times = np.linspace(0, duration, n_steps, endpoint=False)

        self.pub = rospy.Publisher("/preview/joint_states", JointState, queue_size=10)
        self.is_playing = False
        self.current_index = 0

        # Print trajectory info
        q0 = np.array(config["q0"])
        print(f"Loaded: {trajectory_path}")
        print(f"  Duration: {duration}s, FPS: {fps}, steps: {n_steps}")
        print(f"  q0 (deg): {np.rad2deg(q0).round(1).tolist()}")
        q_min = np.rad2deg(self.positions.min(axis=0))
        q_max = np.rad2deg(self.positions.max(axis=0))
        print(f"  q_min (deg): {q_min.round(1).tolist()}")
        print(f"  q_max (deg): {q_max.round(1).tolist()}")
        if "condition_number" in data:
            print(f"  Condition number: {data['condition_number']:.4f}")

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
        self.current_index = 0

        print("Playing trajectory...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown() and self.current_index < len(self.times):
            elapsed = (rospy.Time.now() - start_time).to_sec()

            while (
                self.current_index < len(self.times) - 1
                and self.times[self.current_index] < elapsed
            ):
                self.current_index += 1

            self.publish_joint_state(self.current_index)

            progress = self.current_index / len(self.times) * 100
            t = self.times[self.current_index]
            q_deg = np.rad2deg(self.positions[self.current_index])
            sys.stdout.write(
                f"\r  t={t:5.2f}s | q1={q_deg[0]:7.1f} q2={q_deg[1]:7.1f}"
                f" q3={q_deg[2]:7.1f} | {progress:5.1f}%"
            )
            sys.stdout.flush()
            rate.sleep()

        print("\n  Playback complete!")
        self.is_playing = False

    def interactive_mode(self):
        print("\n" + "=" * 50)
        print("Optimized Trajectory Preview")
        print("=" * 50)
        print("Commands:")
        print("  [Enter] - Play trajectory")
        print("  [s]     - Show start position")
        print("  [e]     - Show end position")
        print("  [r]     - Replay")
        print("  [q]     - Quit")
        print("=" * 50)

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
            elif cmd == "q":
                print("Exiting...")
                break
            else:
                print(f"Unknown command: {cmd}")


def main():
    trajectory_path = DEFAULT_TRAJECTORY
    if len(sys.argv) > 1:
        trajectory_path = sys.argv[1]

    try:
        node = OptimizedTrajectoryPreviewNode(trajectory_path)
        node.interactive_mode()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
