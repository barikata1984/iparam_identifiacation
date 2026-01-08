#!/usr/bin/env python3
import rospy
import numpy as np
import sys
from geometry_msgs.msg import Vector3
from scipy import constants
from sensor_msgs.msg import JointState
from iparam_identification.dynamics_utils import compute_body_twist_and_derivative
from iparam_identification.numerical_differentiator import NumericalDifferentiator
from ur_pykdl.ur_pykdl import ur_kinematics
from pymlg import SE3, SO3

from iparam_identification.joint_state_utils import process_joint_state_msg


class WristEndKinematicsNode:
    def __init__(self):
        rospy.init_node("wrist_end_kinematics", anonymous=False)

        self.base_link = rospy.get_param("~base_link", "base_link")
        self.ee_link = rospy.get_param("~ee_link", "tool0")
        self.cutoff_freq = rospy.get_param("~cutoff_freq", 10.0)
        self.gravity = np.array(rospy.get_param("~gravity", [0.0, 0.0, -constants.g]))

        # Publishers
        self.pub_lv = rospy.Publisher("~lv_tool0", Vector3, queue_size=10)
        self.pub_av = rospy.Publisher("~av_tool0", Vector3, queue_size=10)
        self.pub_la = rospy.Publisher("~la_tool0", Vector3, queue_size=10)
        self.pub_aa = rospy.Publisher("~aa_tool0", Vector3, queue_size=10)

        self.kinematics = ur_kinematics(base_link=self.base_link, ee_link=self.ee_link)
        self.vel_diff = NumericalDifferentiator(cutoff_freq=self.cutoff_freq)

        self.joint_positions = None
        self.joint_velocities = None

        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        rospy.loginfo("Wrist End Kinematics Node Started.")

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
        rate = rospy.Rate(500)  # High frequency for smooth publishing

        while not rospy.is_shutdown():
            if self.joint_positions is not None:
                t = rospy.get_time()

                # Get velocity in base frame (dp, w)
                vel_base_tool0 = np.array(
                    self.kinematics.forward_velocity(
                        self.joint_positions, self.joint_velocities
                    )
                )
                # Debug output
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

                lv_tool0 = rot_tool0_base @ lv_base_tool0
                av_tool0 = rot_tool0_base @ av_base_tool0

                # Coordinate-transform the accelecations from base to tool0
                la_base_tool0_kinematic = acc_base_tool0[:3]
                aa_base_tool0 = acc_base_tool0[3:]

                # Proper acceleration: a_proper = a_kinematic - g
                # This adds +9.81 upwards if g = [0, 0, -constants.g]
                # NOTE: Do NOT use -= operator on a slice of acc_base_tool0, as it
                # modifies the differentiator's internal state in-place!
                la_base_tool0 = la_base_tool0_kinematic - self.gravity

                # Angular acceleration: R^T * alpha_s
                aa_tool0 = rot_tool0_base @ aa_base_tool0

                # Linear acceleration: R^T * a_proper - w_b x v_b (Coriolis/Convective term)
                coriolis_term = (
                    rot_tool0_base @ SO3.wedge(av_base_tool0) @ lv_base_tool0
                )
                la_tool0 = rot_tool0_base @ la_base_tool0 - coriolis_term

                # Publish messages
                self.pub_lv.publish(Vector3(*lv_tool0))
                self.pub_av.publish(Vector3(*av_tool0))
                self.pub_la.publish(Vector3(*la_tool0))
                self.pub_aa.publish(Vector3(*aa_tool0))

            rate.sleep()


if __name__ == "__main__":
    try:
        node = WristEndKinematicsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
