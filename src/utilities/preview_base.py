"""Base class for trajectory preview nodes in RViz.

Provides common functionality: JointState publishing, hold loop,
playback with time-indexed positions, and interactive mode.
"""

import sys
import threading
from abc import ABC, abstractmethod

import numpy as np
import rospy
from sensor_msgs.msg import JointState

from .tool0_kinematics import JOINT_ORDER


class TrajectoryPreviewBase(ABC):
    """Base class for RViz trajectory preview nodes.

    Subclasses must implement:
        - load_trajectory(): populate self.times, self.positions, self.velocities
        - print_info(): display trajectory metadata
    """

    def __init__(self, node_name: str):
        rospy.init_node(node_name, anonymous=True)
        self.pub = rospy.Publisher("/preview/joint_states", JointState, queue_size=10)
        self.is_playing = False
        self.speed = 1.0

        # Subclass must set these in load_trajectory()
        self.times: np.ndarray = np.array([])
        self.positions: np.ndarray = np.array([])
        self.velocities: np.ndarray = np.array([])

    def publish_joint_state(self, index: int) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = list(JOINT_ORDER)
        msg.position = self.positions[index].tolist()
        msg.velocity = self.velocities[index].tolist()
        msg.effort = [0.0] * 6
        self.pub.publish(msg)

    def playback(self) -> None:
        self.is_playing = True
        current_index = 0
        duration = self.times[-1]
        playback_duration = duration / self.speed

        print(f"Playing trajectory (speed={self.speed}x, duration={playback_duration:.1f}s)...")
        start_time = rospy.Time.now()
        rate = rospy.Rate(100)

        while not rospy.is_shutdown() and current_index < len(self.times):
            elapsed = (rospy.Time.now() - start_time).to_sec()
            t_traj = elapsed * self.speed

            while current_index < len(self.times) - 1 and self.times[current_index] < t_traj:
                current_index += 1

            self.publish_joint_state(current_index)

            progress = current_index / (len(self.times) - 1) * 100
            t = self.times[current_index]
            q_deg = np.rad2deg(self.positions[current_index])
            sys.stdout.write(
                f"\r  t={t:5.2f}/{duration:.2f}s"
                f" | q1={q_deg[0]:7.1f} q2={q_deg[1]:7.1f} q3={q_deg[2]:7.1f}"
                f" | {progress:5.1f}%"
            )
            sys.stdout.flush()
            rate.sleep()

        print("\n  Playback complete!")
        self.is_playing = False

    def interactive_mode(self) -> None:
        self._print_commands()

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
                hold_index = self._handle_extra_command(cmd, hold_index)

    @abstractmethod
    def _print_commands(self) -> None:
        """Print available commands for interactive mode."""

    def _handle_extra_command(self, cmd: str, hold_index: int) -> int:
        """Handle subclass-specific commands. Return updated hold_index."""
        print(f"Unknown command: {cmd}")
        return hold_index
