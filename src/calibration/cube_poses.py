"""
6-face cube poses for F/T preload calibration.

Defines 6 joint configurations where the tool0 flange Z-axis points in each
of the 6 cardinal directions (+-X, +-Y, +-Z in base frame). These poses are
used to verify that the ft_raw_wrench preload is orientation-invariant.

Pose computation:
    - Pose 1 (-Z): excitation trajectory start pose (known analytically)
    - Poses 2-6: computed via Pinocchio CLIK (Closed-Loop Inverse Kinematics)
      from the same TCP position with different orientations
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pinocchio as pin

from utilities.tool0_kinematics import DEFAULT_URDF_PATH

# Base (reference) pose: excitation trajectory start
# Flange Z = base -Z (pointing down)
_BASE_Q_DEG = np.array([90.0, -90.0, 90.0, -90.0, -90.0, 0.0])
BASE_Q_RAD = np.deg2rad(_BASE_Q_DEG)

# 6 cardinal directions for flange Z-axis in base frame
FACE_DIRECTIONS: dict[str, np.ndarray] = {
    "-Z": np.array([0.0, 0.0, -1.0]),
    "+Z": np.array([0.0, 0.0, 1.0]),
    "+X": np.array([1.0, 0.0, 0.0]),
    "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y": np.array([0.0, 1.0, 0.0]),
    "-Y": np.array([0.0, -1.0, 0.0]),
}


@dataclass
class CubePose:
    """A calibration pose with metadata."""

    label: str
    flange_z_direction: np.ndarray
    joint_angles_rad: np.ndarray

    @property
    def joint_angles_deg(self) -> np.ndarray:
        return np.rad2deg(self.joint_angles_rad)


def _rotation_from_z_axis(z_desired: np.ndarray) -> np.ndarray:
    """Build a rotation matrix whose Z column is z_desired."""
    z = z_desired / np.linalg.norm(z_desired)

    # Pick an arbitrary vector not parallel to z for cross product
    if abs(z[0]) < 0.9:
        aux = np.array([1.0, 0.0, 0.0])
    else:
        aux = np.array([0.0, 1.0, 0.0])

    x = np.cross(aux, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def _solve_ik_clik(
    model: pin.Model,
    data: pin.Data,
    tool0_id: int,
    target_se3: pin.SE3,
    q_init: np.ndarray,
    max_iter: int = 200,
    eps: float = 1e-6,
    dt: float = 0.1,
    damp: float = 1e-6,
) -> np.ndarray | None:
    """Closed-loop inverse kinematics (damped least-squares)."""
    q = q_init.copy()
    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        oMf = data.oMf[tool0_id]
        err = pin.log6(oMf.actInv(target_se3)).vector
        if np.linalg.norm(err) < eps:
            return q
        J = pin.computeFrameJacobian(model, data, q, tool0_id, pin.LOCAL)
        JtJ = J.T @ J + damp * np.eye(model.nv)
        dq = np.linalg.solve(JtJ, J.T @ err)
        q = pin.integrate(model, q, dt * dq)
    return None


def compute_cube_poses(urdf_path: str = DEFAULT_URDF_PATH) -> list[CubePose]:
    """
    Compute 6 cube poses via Pinocchio IK.

    Returns a list of CubePose, one for each cardinal direction.
    Raises RuntimeError if IK fails for any pose.
    """
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    tool0_id = model.getFrameId("tool0")

    # Get reference position from base pose FK
    pin.forwardKinematics(model, data, BASE_Q_RAD)
    pin.updateFramePlacements(model, data)
    ref_position = data.oMf[tool0_id].translation.copy()

    poses: list[CubePose] = []
    for label, z_dir in FACE_DIRECTIONS.items():
        R_desired = _rotation_from_z_axis(z_dir)
        target = pin.SE3(R_desired, ref_position)

        q_sol = _solve_ik_clik(model, data, tool0_id, target, BASE_Q_RAD)
        if q_sol is None:
            raise RuntimeError(f"IK failed for face {label} (flange Z = {z_dir})")

        poses.append(
            CubePose(
                label=label,
                flange_z_direction=z_dir.copy(),
                joint_angles_rad=q_sol,
            )
        )

    return poses
