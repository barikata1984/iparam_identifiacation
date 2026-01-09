#!/usr/bin/env python3
import rospy
import numpy as np
import sys
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from scipy import constants
from sensor_msgs.msg import JointState
from iparam_identification.numerical_differentiator import NumericalDifferentiator
from iparam_identification.wrist_end_kinematics_utils import (
    coordinate_transform_linang_velacc,
    get_pose,
    get_regressor_matrix,
)
from ur_pykdl.ur_pykdl import ur_kinematics

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
        self.pub_regressor = rospy.Publisher(
            "~regressor", Float64MultiArray, queue_size=10
        )

        self.kin = ur_kinematics(base_link=self.base_link, ee_link=self.ee_link)
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
                    self.kin.forward_velocity(
                        self.joint_positions, self.joint_velocities
                    )
                )
                acc_base_tool0 = self.vel_diff.update(vel_base_tool0, t)

                # Get rot_base_tool0 to transform velocity to tool0 frame
                _pose_base_tool0 = self.kin.forward_position_kinematics(
                    self.joint_positions
                )

                rot_tool0_base = get_pose(_pose_base_tool0, inverse=True, only_rot=True)

                lv_tool0, av_tool0, la_tool0, aa_tool0 = (
                    coordinate_transform_linang_velacc(
                        rot_tool0_base, vel_base_tool0, acc_base_tool0, self.gravity
                    )
                )

                # Construct Regressor Matrix (Shape: 6x10)
                regressor = get_regressor_matrix(
                    linear_acc=la_tool0, angular_vel=av_tool0, angular_acc=aa_tool0
                )

                # Publish messages
                self.pub_lv.publish(Vector3(*lv_tool0))
                self.pub_av.publish(Vector3(*av_tool0))
                self.pub_la.publish(Vector3(*la_tool0))
                self.pub_aa.publish(Vector3(*aa_tool0))

                regressor_msg = Float64MultiArray()
                regressor_msg.data = regressor.flatten().tolist()
                self.pub_regressor.publish(regressor_msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = WristEndKinematicsNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
