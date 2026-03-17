"""
Test cube poses: verify that FK of each computed pose gives the expected
flange Z direction in the base frame.
"""

import sys
from pathlib import Path

import numpy as np
import numpy.testing as npt
import pinocchio as pin
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from calibration.cube_poses import (
    BASE_Q_RAD,
    FACE_DIRECTIONS,
    CubePose,
    compute_cube_poses,
)
from utilities.tool0_kinematics import DEFAULT_URDF_PATH


@pytest.fixture(scope="module")
def cube_poses() -> list[CubePose]:
    return compute_cube_poses()


@pytest.fixture(scope="module")
def pinocchio_model():
    model = pin.buildModelFromUrdf(DEFAULT_URDF_PATH)
    data = model.createData()
    tool0_id = model.getFrameId("tool0")
    return model, data, tool0_id


class TestCubePoseComputation:
    def test_returns_six_poses(self, cube_poses: list[CubePose]):
        assert len(cube_poses) == 6

    def test_all_face_directions_covered(self, cube_poses: list[CubePose]):
        labels = {p.label for p in cube_poses}
        assert labels == set(FACE_DIRECTIONS.keys())

    @pytest.mark.parametrize("face", list(FACE_DIRECTIONS.keys()))
    def test_flange_z_matches_expected_direction(
        self,
        face: str,
        cube_poses: list[CubePose],
        pinocchio_model,
    ):
        model, data, tool0_id = pinocchio_model

        pose = next(p for p in cube_poses if p.label == face)
        q = pose.joint_angles_rad

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        R = data.oMf[tool0_id].rotation

        flange_z_actual = R[:, 2]
        npt.assert_allclose(flange_z_actual, pose.flange_z_direction, atol=1e-4)

    @pytest.mark.parametrize("face", list(FACE_DIRECTIONS.keys()))
    def test_tcp_position_near_reference(
        self,
        face: str,
        cube_poses: list[CubePose],
        pinocchio_model,
    ):
        """All poses should have TCP position near the base pose."""
        model, data, tool0_id = pinocchio_model

        # Reference position from base pose
        pin.forwardKinematics(model, data, BASE_Q_RAD)
        pin.updateFramePlacements(model, data)
        ref_pos = data.oMf[tool0_id].translation.copy()

        # Pose position
        pose = next(p for p in cube_poses if p.label == face)
        pin.forwardKinematics(model, data, pose.joint_angles_rad)
        pin.updateFramePlacements(model, data)
        pos = data.oMf[tool0_id].translation

        npt.assert_allclose(pos, ref_pos, atol=1e-3)

    def test_base_pose_is_minus_z(self, pinocchio_model):
        """Verify the base pose gives flange Z = [0, 0, -1]."""
        model, data, tool0_id = pinocchio_model

        pin.forwardKinematics(model, data, BASE_Q_RAD)
        pin.updateFramePlacements(model, data)
        R = data.oMf[tool0_id].rotation

        npt.assert_allclose(R[:, 2], [0.0, 0.0, -1.0], atol=1e-6)

    def test_joint_angles_deg_property(self, cube_poses: list[CubePose]):
        for pose in cube_poses:
            npt.assert_allclose(
                pose.joint_angles_deg,
                np.rad2deg(pose.joint_angles_rad),
            )
