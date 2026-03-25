#!/usr/bin/env python3
"""
Tool0 Kinematics Calculator using Pinocchio.

This module provides the Tool0KinematicsCalculator class for computing
classical velocities and accelerations at the tool0 frame of a UR5e robot.
"""

import numpy as np
import pinocchio as pin

from .numerical_differentiator import NumericalDifferentiator


# Joint order for pinocchio model (UR5e standard)
JOINT_ORDER = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Default URDF path for UR5e
DEFAULT_URDF_PATH = (
    "/root/osx-ur/underlay_ws/src/ur_python_utilities/ur_pykdl/urdf/ur5e.urdf"
)


class Tool0KinematicsCalculator:
    """
    Computes classical velocities and accelerations at the tool0 frame.

    This class uses pinocchio for forward kinematics and provides classical
    (not spatial) accelerations suitable for inertial parameter identification.

    Attributes
    ----------
    model : pinocchio.Model
        The pinocchio robot model
    data : pinocchio.Data
        The pinocchio data structure for computations
    tool0_id : int
        Frame ID of tool0 in the pinocchio model
    acc_differentiator : NumericalDifferentiator
        Differentiator for computing joint accelerations from velocities
    """

    def __init__(self, urdf_path: str = DEFAULT_URDF_PATH, acc_cutoff_freq: float = 10.0):
        """
        Initialize the calculator.

        Parameters
        ----------
        urdf_path : str
            Path to the URDF file
        acc_cutoff_freq : float
            Cutoff frequency for acceleration low-pass filter (Hz)
        """
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.tool0_id = self.model.getFrameId("tool0")

        # Numerical differentiator for joint acceleration (velocity -> acceleration)
        self.acc_differentiator = NumericalDifferentiator(cutoff_freq=acc_cutoff_freq)

    def compute(self, q: np.ndarray, v: np.ndarray, t: float) -> dict:
        """
        Compute tool0 classical velocities and accelerations.

        Parameters
        ----------
        q : np.ndarray
            Joint positions [rad], shape (6,)
        v : np.ndarray
            Joint velocities [rad/s], shape (6,) - directly from /joint_states
        t : float
            Timestamp [s]

        Returns
        -------
        dict
            Dictionary containing:
            - angular_velocity: ω [rad/s] in tool0 frame
            - linear_velocity: ṗ [m/s] in tool0 frame
            - angular_acceleration: ω̇ [rad/s²] in tool0 frame
            - linear_acceleration: p̈ [m/s²] in tool0 frame (classical)
            - joint_position: q [rad]
            - joint_velocity: v [rad/s]
            - joint_acceleration: a [rad/s²]
        """
        # Compute joint acceleration via numerical differentiation
        a = self.acc_differentiator.update(v, t)

        # Forward kinematics with position, velocity, and acceleration
        pin.forwardKinematics(self.model, self.data, q, v, a)
        pin.updateFramePlacements(self.model, self.data)

        # Get spatial velocity (twist) in LOCAL frame
        twist = pin.getFrameVelocity(
            self.model, self.data, self.tool0_id, pin.ReferenceFrame.LOCAL
        )

        # Get classical acceleration in LOCAL frame
        # This automatically applies the ω × v correction for linear acceleration
        classical_acc = pin.getFrameClassicalAcceleration(
            self.model, self.data, self.tool0_id, pin.ReferenceFrame.LOCAL
        )

        return {
            # Velocities (in LOCAL frame, spatial = classical for velocity)
            "angular_velocity": twist.angular.copy(),  # ω [rad/s]
            "linear_velocity": twist.linear.copy(),  # ṗ [m/s]
            # Classical accelerations (position's 2nd time derivative)
            "angular_acceleration": classical_acc.angular.copy(),  # ω̇ [rad/s²]
            "linear_acceleration": classical_acc.linear.copy(),  # p̈ [m/s²]
            # Raw joint data (for debugging)
            "joint_position": q.copy(),
            "joint_velocity": v.copy(),
            "joint_acceleration": a.copy(),
        }

    def compute_with_gravity(
        self,
        q: np.ndarray,
        v: np.ndarray,
        t: float,
        gravity: np.ndarray = np.array([0.0, 0.0, -9.81]),
    ) -> dict:
        """
        Compute tool0 kinematics with gravity compensation for proper acceleration.

        This method computes the proper acceleration (what an accelerometer would
        measure) by adding the gravity vector transformed to the LOCAL frame.

        For inertial parameter identification, the F/T sensor measures:
            F = m * a_proper
        where:
            a_proper = a_kinematic + g_local

        Parameters
        ----------
        q : np.ndarray
            Joint positions [rad], shape (6,)
        v : np.ndarray
            Joint velocities [rad/s], shape (6,) - directly from /joint_states
        t : float
            Timestamp [s]
        gravity : np.ndarray
            Gravity vector in base/world frame [m/s²], default [0, 0, -9.81]

        Returns
        -------
        dict
            Dictionary containing all fields from compute() plus:
            - proper_linear_acceleration: a_kinematic + g_local [m/s²]
            - gravity_local: gravity vector in tool0 frame [m/s²]
            - rotation_tool0_base: rotation matrix from base to tool0
        """
        # Compute kinematic quantities
        result = self.compute(q, v, t)

        # Get rotation matrix from base to tool0 (LOCAL frame)
        oMf = self.data.oMf[self.tool0_id]
        R_local_world = oMf.rotation.T  # tool0 <- base rotation

        # Transform gravity to LOCAL (tool0) frame
        gravity_local = R_local_world @ gravity

        # Proper acceleration = kinematic acceleration + gravity (in local frame)
        # This is what an accelerometer would measure, and corresponds to
        # the (a - g) term in the regressor matrix when g is defined as [0,0,-9.81]
        proper_linear_acc = result["linear_acceleration"] + gravity_local

        # Add gravity-related quantities to result
        result["proper_linear_acceleration"] = proper_linear_acc
        result["gravity_local"] = gravity_local
        result["rotation_tool0_base"] = R_local_world

        return result

    def reset(self):
        """Reset the numerical differentiator state."""
        self.acc_differentiator.reset()


def reorder_joint_state(names: list, positions: list, velocities: list) -> tuple:
    """
    Reorder joint state arrays to match pinocchio model's expected order.

    Parameters
    ----------
    names : list
        Joint names from /joint_states message
    positions : list
        Joint positions from /joint_states message
    velocities : list
        Joint velocities from /joint_states message

    Returns
    -------
    tuple
        (q, v) - reordered joint positions and velocities as numpy arrays
    """
    q = np.zeros(6)
    v = np.zeros(6)

    for i, name in enumerate(JOINT_ORDER):
        if name in names:
            idx = names.index(name)
            q[i] = positions[idx]
            v[i] = velocities[idx]

    return q, v
