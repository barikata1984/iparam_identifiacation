#!/usr/bin/env python3
import os
import rospy
import numpy as np
import atexit
import matplotlib

# Use interactive backend for popup plots if DISPLAY is available
if os.environ.get("DISPLAY"):
    try:
        matplotlib.use("TkAgg")
    except Exception:
        pass
import matplotlib.pyplot as plt

# Ensure all figures are closed on main thread before GC runs,
# preventing tkinter "main thread is not in main loop" errors at shutdown.
atexit.register(plt.close, "all")
import sys
import select
import termios
import tty
from sensor_msgs.msg import JointState
from iparam_identification.dynamics_utils import compute_body_twist_and_derivative
from iparam_identification.numerical_differentiator import NumericalDifferentiator
from ur_pykdl.ur_pykdl import ur_kinematics
from pymlg import SE3, SO3

from iparam_identification.joint_state_utils import process_joint_state_msg


class VerifyTwistNode:
    def __init__(self):
        rospy.init_node("verify_twist_node", anonymous=True, disable_signals=True)

        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link = rospy.get_param("~ee_link", "tool0")
        self.cutoff_freq = rospy.get_param("~cutoff_freq", 10.0)

        self.kinematics = ur_kinematics(base_link=self.base_link, ee_link=self.ee_link)
        self.twist_diff = NumericalDifferentiator(cutoff_freq=self.cutoff_freq)

        self.joint_positions = None
        self.joint_velocities = None

        self.time_history = []
        self.linear_twist_history = []
        self.angular_twist_history = []
        self.euler_history = []

        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        rospy.loginfo("Verify Twist Node Started.")
        rospy.loginfo("Press 'Esc' to stop capture and plot results.")

    def joint_state_cb(self, msg):
        target_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        pos, vel = process_joint_state_msg(msg, target_joints)
        if pos is not None:
            self.joint_positions = pos
            self.joint_velocities = vel

    def run(self):
        rate = rospy.Rate(50)
        start_time = rospy.get_time()

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0)[0]:
                    c = sys.stdin.read(1)
                    if c == "\x1b":
                        rospy.loginfo("Esc pressed. Stopping capture...")
                        break

                if self.joint_positions is not None:
                    t = rospy.get_time()

                    # Get twist and its derivative
                    twist_base_tool0 = np.array(
                        self.kinematics.forward_velocity(
                            self.joint_positions, self.joint_velocities
                        )
                    )
                    dtwist_base_tool0 = self.twist_diff.update(twist_base_tool0, t)

                    # Get pose_base_tool0 to transform the twist's reference frame from base to tool0
                    _pose_base_tool0 = self.kinematics.forward_position_kinematics(
                        self.joint_positions
                    )
                    trans_base_tool0 = np.array(_pose_base_tool0[:3])
                    quat_base_tool0 = np.array(_pose_base_tool0[3:])
                    # ur_pykdl returns [x, y, z, w] while pymlg.SO3 use "wxyz" as default
                    rot_base_tool0 = SO3.from_quat(quat_base_tool0, order="xyzw")
                    pose_base_tool0 = SE3.from_components(
                        rot_base_tool0, trans_base_tool0
                    )
                    pose_tool0_base = SE3.inverse(pose_base_tool0)
                    Ad_tool0_base = SE3.adjoint(pose_tool0_base)
                    twist_tool0_tool0 = Ad_tool0_base @ twist_base_tool0

                    # Get the twist's lie bracket to get the twist's derivative
                    ad_tool0_tool0 = SE3.adjoint_algebra(SE3.wedge(twist_tool0_tool0))

                    dtwist_tool0_tool0 = (
                        ad_tool0_tool0 @ twist_tool0_tool0
                        + Ad_tool0_base @ dtwist_base_tool0
                    )

                    # `order="321"` corresponds to Intrinsic-XYZ (Roll-Pitch-Yaw)
                    euler = SO3.to_euler(rot_base_tool0, order="321")
                    time = t - start_time

                    # Store data for verifycation
                    self.linear_twist_history.append(twist_tool0_tool0[:3])
                    self.angular_twist_history.append(twist_tool0_tool0[3:])
                    self.euler_history.append(euler)
                    self.time_history.append(time)

                    np.set_printoptions(precision=3, suppress=True)
                    if int((time) * 10) % 10 == 0:
                        # Log Twist (Base Frame)
                        # v = twist_base_tool0[:3]
                        # w = twist_base_tool0[3:]
                        v = twist_tool0_tool0[:3]
                        w = twist_tool0_tool0[3:]
                        rospy.loginfo(
                            f"Twist Linear  (Tool0) [vx, vy, vz]: [{v[0]:6.3f}, {v[1]:6.3f}, {v[2]:6.3f}]"
                        )
                        rospy.loginfo(
                            f"Twist Angular (Tool0) [wx, wy, wz]: [{w[0]:6.3f}, {w[1]:6.3f}, {w[2]:6.3f}]"
                        )

                        # Log Twist Derivative (Tool0 Frame)
                        dv_tool0 = dtwist_tool0_tool0[:3]
                        dw_tool0 = dtwist_tool0_tool0[3:]
                        # rospy.loginfo(
                        #    f"DTwist Linear (Tool0) [ax, ay, az]: [{dv_tool0[0]:6.3f}, {dv_tool0[1]:6.3f}, {dv_tool0[2]:6.3f}]"
                        # )
                        # rospy.loginfo(
                        #    f"DTwist Angular (Tool0) [alphax, alphay, alphaz]: [{dw_tool0[0]:6.3f}, {dw_tool0[1]:6.3f}, {dw_tool0[2]:6.3f}]"
                        # )

                        # Log Euler angles in degrees
                        # Validated experimentally as Intrinsic XYZ (Roll->Pitch->Yaw) despite order='321' arg
                        e_deg = np.degrees(euler)
                        rospy.loginfo(
                            f"Tool0 Euler (Intrinsic-XYZ) [Roll, Pitch, Yaw]: [{e_deg[0]:6.2f}, {e_deg[1]:6.2f}, {e_deg[2]:6.2f}]"
                        )
                        rospy.loginfo("-" * 30)

                rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error: {e}")
            import traceback

            rospy.logerr(traceback.format_exc())

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        self.plot_results()

    def plot_results(self):
        if not self.time_history:
            rospy.logwarn("No data collected.")
            return

        rospy.loginfo("Plotting results...")

        time_data = np.array(self.time_history)
        linear_data = np.array(self.linear_twist_history)
        angular_data = np.array(self.angular_twist_history)
        euler_data = np.array(self.euler_history)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

        ax1.plot(time_data, linear_data[:, 0], label="v_x")
        ax1.plot(time_data, linear_data[:, 1], label="v_y")
        ax1.plot(time_data, linear_data[:, 2], label="v_z")
        ax1.set_title("Linear Velocity (Base Frame)")
        ax1.set_ylabel("Velocity [m/s]")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(time_data, angular_data[:, 0], label="w_x")
        ax2.plot(time_data, angular_data[:, 1], label="w_y")
        ax2.plot(time_data, angular_data[:, 2], label="w_z")
        ax2.set_title("Angular Velocity (Base Frame)")
        ax2.set_ylabel("Velocity [rad/s]")
        ax2.legend()
        ax2.grid(True)

        # Plot Euler Angles (converted to degrees for readability)
        ax3.plot(time_data, np.degrees(euler_data[:, 0]), label="Roll (X)")
        ax3.plot(time_data, np.degrees(euler_data[:, 1]), label="Pitch (Y)")
        ax3.plot(time_data, np.degrees(euler_data[:, 2]), label="Yaw (Z)")
        ax3.set_title("Euler Angles (Base Frame) [deg]")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Angle [deg]")
        ax3.legend()
        ax3.grid(True)

        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    try:
        node = VerifyTwistNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
