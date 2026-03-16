#!/usr/bin/env python3
"""
Preview trajectory in RViz.

This script publishes joint states to a preview topic so you can visualize
the planned trajectory in RViz without moving the real robot.

Usage:
    1. Launch RViz with the robot model
    2. Run this script: python3 preview_trajectory_rviz.py
    3. In RViz, add a "RobotModel" display and set:
       - Robot Description: robot_description
       - TF Prefix: preview (if using separate TF tree)
       OR use the /preview/joint_states topic directly

Press Enter to start playback, 'q' to quit, 'r' to replay.
"""

import json
import sys
import numpy as np
import threading

import rospy
from sensor_msgs.msg import JointState


class TrajectoryPreviewNode:
    """Preview trajectory in RViz by publishing to joint_states topic."""

    def __init__(self, trajectory_path: str, use_preview_topic: bool = True):
        """
        Initialize the preview node.

        Parameters
        ----------
        trajectory_path : str
            Path to the trajectory JSON file
        use_preview_topic : bool
            If True, publish to /preview/joint_states (won't affect real robot)
            If False, publish to /joint_states (will override real robot display)
        """
        rospy.init_node("trajectory_preview", anonymous=True)

        # Load trajectory
        with open(trajectory_path, "r") as f:
            self.trajectory_data = json.load(f)

        self.joint_names = self.trajectory_data["joint_names"]
        self.times = self.trajectory_data["trajectory"]["times"]
        self.positions_deg = self.trajectory_data["trajectory"]["positions_deg"]
        self.velocities_deg_s = self.trajectory_data["trajectory"]["velocities_deg_s"]

        # Convert to radians
        self.positions_rad = np.deg2rad(self.positions_deg)
        self.velocities_rad_s = np.deg2rad(self.velocities_deg_s)

        # Moving joint info
        self.moving_joint_index = self.trajectory_data.get("moving_joint_index", 0)
        self.moving_joint_name = self.trajectory_data.get("moving_joint_name", "shoulder_pan_joint")

        # Publisher
        topic_name = "/preview/joint_states" if use_preview_topic else "/joint_states"
        self.pub = rospy.Publisher(topic_name, JointState, queue_size=10)

        # Control
        self.playback_speed = 1.0
        self.is_playing = False
        self.current_index = 0

        print(f"Loaded trajectory: {trajectory_path}")
        print(f"  Joint names: {self.joint_names}")
        print(f"  Total points: {len(self.times)}")
        print(f"  Duration: {self.times[-1]:.2f}s")
        print(f"  Publishing to: {topic_name}")
        print()

    def publish_joint_state(self, index: int):
        """Publish joint state at given index."""
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.joint_names
        msg.position = self.positions_rad[index].tolist()
        msg.velocity = self.velocities_rad_s[index].tolist()
        msg.effort = [0.0] * 6

        self.pub.publish(msg)

    def playback(self):
        """Play the trajectory."""
        self.is_playing = True
        self.current_index = 0

        print("Playing trajectory...")
        print("  Press Ctrl+C to stop")

        start_time = rospy.Time.now()
        rate = rospy.Rate(100)  # 100 Hz for smooth visualization

        while not rospy.is_shutdown() and self.current_index < len(self.times):
            elapsed = (rospy.Time.now() - start_time).to_sec() * self.playback_speed

            # Find the appropriate index for current time
            while (
                self.current_index < len(self.times) - 1
                and self.times[self.current_index] < elapsed
            ):
                self.current_index += 1

            self.publish_joint_state(self.current_index)

            # Progress display
            progress = self.current_index / len(self.times) * 100
            t = self.times[self.current_index]
            ji = self.moving_joint_index
            pos = self.positions_deg[self.current_index][ji]  # Moving joint position
            sys.stdout.write(f"\r  Time: {t:6.2f}s | J{ji+1}: {pos:7.2f}° | Progress: {progress:5.1f}%")
            sys.stdout.flush()

            rate.sleep()

        print("\n  Playback complete!")
        self.is_playing = False

    def hold_position(self, index: int):
        """Hold position at given index."""
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_joint_state(index)
            rate.sleep()

    def interactive_mode(self):
        """Interactive control mode."""
        print("\n" + "=" * 60)
        print("Trajectory Preview - Interactive Mode")
        print("=" * 60)
        ji = self.moving_joint_index
        print(f"Moving joint: J{ji+1} ({self.moving_joint_name})")
        print(f"  From: {self.trajectory_data['start_position_deg'][ji]}°")
        print(f"  To:   {self.trajectory_data['end_position_deg'][ji]}°")
        print(f"Start position (all joints): {self.trajectory_data['start_position_deg']}")
        print(f"End position (all joints):   {self.trajectory_data['end_position_deg']}")
        print()
        print("Commands:")
        print("  [Enter] - Play trajectory")
        print("  [s]     - Show start position")
        print("  [e]     - Show end position")
        print("  [r]     - Replay")
        print("  [q]     - Quit")
        print("=" * 60)

        # Start a thread to continuously publish current position
        hold_thread = None
        current_hold_index = 0

        def hold_loop():
            rate = rospy.Rate(30)
            while not rospy.is_shutdown() and not self.is_playing:
                self.publish_joint_state(current_hold_index)
                rate.sleep()

        while not rospy.is_shutdown():
            # Start holding thread if not playing
            if hold_thread is None or not hold_thread.is_alive():
                if not self.is_playing:
                    hold_thread = threading.Thread(target=hold_loop, daemon=True)
                    hold_thread.start()

            try:
                cmd = input("\nCommand: ").strip().lower()
            except EOFError:
                break

            if cmd == "" or cmd == "p":
                # Play
                self.playback()
                current_hold_index = len(self.times) - 1  # Hold at end after playback

            elif cmd == "s":
                # Show start
                print("Showing start position...")
                current_hold_index = 0
                self.publish_joint_state(0)

            elif cmd == "e":
                # Show end
                print("Showing end position...")
                current_hold_index = len(self.times) - 1
                self.publish_joint_state(current_hold_index)

            elif cmd == "r":
                # Replay (same as play)
                self.playback()
                current_hold_index = len(self.times) - 1

            elif cmd == "q":
                print("Exiting...")
                break

            else:
                print(f"Unknown command: {cmd}")


def main():
    trajectory_path = "/root/osx-ur/catkin_ws/src/iparam_identification/results/j1_constant_velocity_trajectory.json"

    # Check for command line argument
    if len(sys.argv) > 1:
        trajectory_path = sys.argv[1]

    try:
        node = TrajectoryPreviewNode(trajectory_path, use_preview_topic=True)
        node.interactive_mode()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted.")


if __name__ == "__main__":
    main()
