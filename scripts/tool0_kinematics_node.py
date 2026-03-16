#!/usr/bin/env python3
"""
Tool0 Kinematics Node using Pinocchio.

This node publishes tool0 frame velocities, accelerations, and the regressor matrix
for inertial parameter identification. It uses pinocchio for forward kinematics
instead of ur_pykdl.

Published Topics:
    ~/lv_tool0 (Vector3): Linear velocity in tool0 frame [m/s]
    ~/av_tool0 (Vector3): Angular velocity in tool0 frame [rad/s]
    ~/la_tool0 (Vector3): Linear proper acceleration in tool0 frame [m/s²]
    ~/aa_tool0 (Vector3): Angular acceleration in tool0 frame [rad/s²]
    ~/regressor (Float64MultiArray): Flattened 6x10 regressor matrix

Subscribed Topics:
    /joint_states (JointState): Robot joint states

Parameters:
    ~cutoff_freq (float): Cutoff frequency for numerical differentiation filter [Hz]
    ~gravity (list): Gravity vector in base frame [m/s²], default [0, 0, -9.81]
"""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from utilities.tool0_kinematics import JOINT_ORDER, Tool0KinematicsCalculator, reorder_joint_state

# Set for O(1) membership check in callback
_UR_JOINT_NAMES = set(JOINT_ORDER)
from utilities.wrist_end_kinematics_utils import get_regressor_matrix


class Tool0KinematicsNode:
    """ROS node for publishing tool0 kinematics using pinocchio."""

    def __init__(self):
        rospy.init_node("tool0_kinematics", anonymous=False)

        # Parameters
        self.cutoff_freq = rospy.get_param("~cutoff_freq", 10.0)
        gravity_list = rospy.get_param("~gravity", [0.0, 0.0, -9.81])
        self.gravity = np.array(gravity_list)

        # Initialize calculator
        self.calculator = Tool0KinematicsCalculator(acc_cutoff_freq=self.cutoff_freq)

        # Publishers (same interface as wrist_end_kinematics_node)
        self.pub_lv = rospy.Publisher("~lv_tool0", Vector3, queue_size=10)
        self.pub_av = rospy.Publisher("~av_tool0", Vector3, queue_size=10)
        self.pub_la = rospy.Publisher("~la_tool0", Vector3, queue_size=10)
        self.pub_aa = rospy.Publisher("~aa_tool0", Vector3, queue_size=10)
        self.pub_regressor = rospy.Publisher("~regressor", Float64MultiArray, queue_size=10)

        # Subscriber
        self.sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        rospy.loginfo("Tool0 Kinematics Node (Pinocchio) Started.")
        rospy.loginfo(f"  Cutoff frequency: {self.cutoff_freq} Hz")
        rospy.loginfo(f"  Gravity: {self.gravity}")

    def joint_state_cb(self, msg: JointState):
        """Process joint state and publish kinematics."""
        # Ignore non-UR messages (e.g. Robotiq gripper publishes 1-joint states)
        if not _UR_JOINT_NAMES.issubset(msg.name):
            return

        # Reorder joint state to match pinocchio model
        q, v = reorder_joint_state(
            list(msg.name), list(msg.position), list(msg.velocity)
        )

        # Get timestamp
        t = msg.header.stamp.to_sec()

        # Compute kinematics with gravity compensation
        result = self.calculator.compute_with_gravity(q, v, t, self.gravity)

        # Extract values
        lv = result["linear_velocity"]
        av = result["angular_velocity"]
        la = result["proper_linear_acceleration"]  # Use proper acceleration
        aa = result["angular_acceleration"]

        # Compute regressor matrix
        regressor = get_regressor_matrix(
            linear_acc=la, angular_vel=av, angular_acc=aa
        )

        # Publish
        self.pub_lv.publish(Vector3(*lv))
        self.pub_av.publish(Vector3(*av))
        self.pub_la.publish(Vector3(*la))
        self.pub_aa.publish(Vector3(*aa))

        regressor_msg = Float64MultiArray()
        regressor_msg.data = regressor.flatten().tolist()
        self.pub_regressor.publish(regressor_msg)


if __name__ == "__main__":
    try:
        node = Tool0KinematicsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
