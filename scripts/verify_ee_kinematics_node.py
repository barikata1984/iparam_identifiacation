#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
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


class VerifyEEKinematicsNode:
    def __init__(self):
        rospy.init_node(
            "verify_ee_kinematics_node", anonymous=True, disable_signals=True
        )

        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link = rospy.get_param("~ee_link", "tool0")
        self.cutoff_freq = rospy.get_param("~cutoff_freq", 10.0)

        self.kinematics = ur_kinematics(base_link=self.base_link, ee_link=self.ee_link)
        self.vel_diff = NumericalDifferentiator(cutoff_freq=self.cutoff_freq)

        self.joint_positions = None
        self.joint_velocities = None

        self.time_history = []
        self.linear_vel_history = []
        self.angular_vel_history = []
        self.euler_history = []

        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        rospy.loginfo("Verify EE Kinematics Node Started.")
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

                    # Get velocity in base frame (dp, w)
                    vel_base_tool0 = np.array(
                        self.kinematics.forward_velocity(
                            self.joint_positions, self.joint_velocities
                        )
                    )
                    acc_base_tool0 = self.vel_diff.update(vel_base_tool0, t)

                    # Get pose_base_tool0 to transform velocity to tool0 frame
                    _pose_base_tool0 = self.kinematics.forward_position_kinematics(
                        self.joint_positions
                    )
                    quat_base_tool0 = np.array(_pose_base_tool0[3:])
                    # ur_pykdl returns [x, y, z, w] while pymlg.SO3 use "wxyz" as default
                    rot_base_tool0 = SO3.from_quat(quat_base_tool0, order="xyzw")
                    rot_tool0_base = rot_base_tool0.T  # Transpose (inverse)

                    # Coordinate-transform the velocities from base to tool0
                    lv_base_tool0 = vel_base_tool0[:3]
                    av_base_tool0 = vel_base_tool0[3:]
                    # =====================================================
                    lv_tool0 = rot_tool0_base @ lv_base_tool0
                    av_tool0 = rot_tool0_base @ av_base_tool0

                    # Coordinate-transform the accelecations from base to tool0
                    if acc_base_tool0 is not None:
                        la_base_tool0 = acc_base_tool0[:3]
                        aa_base_tool0 = acc_base_tool0[3:]
                        # Get angular acceleration first since it's easier
                        aa_tool0 = rot_tool0_base @ aa_base_tool0
                        # Then get linear acceleration first since it's easier
                        coriolis_term = (
                            rot_tool0_base @ SO3.wedge(av_base_tool0) @ lv_base_tool0
                        )
                        la_tool0 = rot_tool0_base @ la_base_tool0 - coriolis_term

                    else:
                        la_tool0 = np.zeros(3)
                        aa_tool0 = np.zeros(3)

                    # `order="321"` corresponds to Intrinsic-XYZ (Roll-Pitch-Yaw)
                    euler = SO3.to_euler(rot_base_tool0, order="321")
                    time = t - start_time

                    # Store Body Twist data for verification
                    self.linear_vel_history.append(lv_tool0)
                    self.angular_vel_history.append(av_tool0)
                    self.euler_history.append(euler)
                    self.time_history.append(time)

                    np.set_printoptions(precision=3, suppress=True)
                    if int((time) * 10) % 10 == 0:
                        # Log Body Twist
                        rospy.loginfo(
                            f"linvel_tool0: [{lv_tool0[0]:6.3f}, {lv_tool0[1]:6.3f}, {lv_tool0[2]:6.3f}]"
                        )
                        rospy.loginfo(
                            f"angvel_tool0: [{av_tool0[0]:6.3f}, {av_tool0[1]:6.3f}, {av_tool0[2]:6.3f}]"
                        )

                        # Log Body Twist Derivative
                        rospy.loginfo(
                            f"linacc_tool0: [{la_tool0[0]:6.3f}, {la_tool0[1]:6.3f}, {la_tool0[2]:6.3f}]"
                        )
                        rospy.loginfo(
                            f"angacc_tool0: [{aa_tool0[0]:6.3f}, {aa_tool0[1]:6.3f}, {aa_tool0[2]:6.3f}]"
                        )

                        # Log Euler angles in degrees
                        # Validated experimentally as Intrinsic XYZ (Roll->Pitch->Yaw) despite order='321' arg
                        e_deg = np.degrees(euler)
                        rospy.loginfo(
                            f"Tool0 Intrinsic-XYZ Euler (RPY): [{e_deg[0]:6.2f}, {e_deg[1]:6.2f}, {e_deg[2]:6.2f}]"
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
        linear_data = np.array(self.linear_vel_history)
        angular_data = np.array(self.angular_vel_history)
        euler_data = np.array(self.euler_history)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

        ax1.plot(time_data, linear_data[:, 0], label="v_x")
        ax1.plot(time_data, linear_data[:, 1], label="v_y")
        ax1.plot(time_data, linear_data[:, 2], label="v_z")
        ax1.set_title("linvel_tool0")
        ax1.set_ylabel("Velocity [m/s]")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(time_data, angular_data[:, 0], label="w_x")
        ax2.plot(time_data, angular_data[:, 1], label="w_y")
        ax2.plot(time_data, angular_data[:, 2], label="w_z")
        ax2.set_title("angvel_tool0")
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
        node = VerifyEEKinematicsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
