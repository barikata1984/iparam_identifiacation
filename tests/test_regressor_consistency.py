#!/usr/bin/env python3
"""Verify wrist_end_kinematics_utils.get_regressor_matrix matches dynamics_utils."""

import sys
from pathlib import Path

import numpy as np
import pytest

# Add src to path
SRC_DIR = Path(__file__).resolve().parent.parent / "src"
sys.path.insert(0, str(SRC_DIR))

from utilities.dynamics_utils import get_regressor_matrix as regressor_lynch_park
from utilities.wrist_end_kinematics_utils import get_regressor_matrix as regressor_wrist


def _build_classical_inputs(omega, alpha, a_proper):
    """Convert classical quantities to the twist/dtwist format for dynamics_utils.

    dynamics_utils expects spatial twist [v, w] and spatial acceleration [dv, dw].
    For a body-fixed frame at the origin (tool0), with classical inputs:
        twist = [v, w]  where v = linear velocity (zero for static test)
        dtwist = [dv, dw]  where dv includes gravity via proper acceleration

    The relationship between classical and spatial linear acceleration:
        a_classical = dv + w x v
    For v=0: a_classical = dv, so dv = a_proper directly.
    """
    v = np.zeros(3)
    twist = np.concatenate([v, omega])
    dtwist = np.concatenate([a_proper, alpha])
    return twist, dtwist


# Test cases: (omega, alpha, a_proper, description)
TEST_CASES = [
    # Static with gravity
    (np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, -9.81]),
     "static, gravity along -z"),
    # Pure rotation around x
    (np.array([1, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, -9.81]),
     "rotation about x, gravity -z"),
    # Pure rotation around y
    (np.array([0, 1, 0]), np.array([0, 0, 0]), np.array([0, 0, -9.81]),
     "rotation about y, gravity -z"),
    # Pure rotation around z
    (np.array([0, 0, 1]), np.array([0, 0, 0]), np.array([0, 0, -9.81]),
     "rotation about z, gravity -z"),
    # Angular acceleration only
    (np.array([0, 0, 0]), np.array([1, 2, 3]), np.array([0, 0, -9.81]),
     "angular acceleration only"),
    # Mixed rotation + acceleration
    (np.array([0.5, -0.3, 0.8]), np.array([1.2, -0.5, 0.7]), np.array([0.3, -0.6, -9.5]),
     "general mixed motion"),
    # High angular velocity
    (np.array([2.0, 1.5, -1.0]), np.array([0.1, -0.2, 0.3]), np.array([1.0, 2.0, -8.0]),
     "high angular velocity"),
    # Near-zero values
    (np.array([0.001, -0.002, 0.003]), np.array([0.01, 0.02, -0.01]),
     np.array([0.01, -0.02, -9.81]),
     "near-zero angular motion"),
]


@pytest.mark.parametrize("omega,alpha,a_proper,desc", TEST_CASES)
def test_regressor_matches_dynamics_utils(omega, alpha, a_proper, desc):
    """Verify wrist_end regressor matches Lynch & Park (dynamics_utils) regressor."""
    # wrist_end_kinematics_utils: classical inputs
    R_wrist = regressor_wrist(a_proper, omega, alpha)

    # dynamics_utils: twist/dtwist inputs (with v=0)
    twist, dtwist = _build_classical_inputs(omega, alpha, a_proper)
    R_lynch = regressor_lynch_park(twist, dtwist)

    np.testing.assert_allclose(
        R_wrist, R_lynch, atol=1e-12,
        err_msg=f"Regressor mismatch for case: {desc}",
    )


def test_known_static_case():
    """Verify mass column equals proper acceleration for static case."""
    a_proper = np.array([0.5, -0.3, -9.81])
    omega = np.zeros(3)
    alpha = np.zeros(3)

    R = regressor_wrist(a_proper, omega, alpha)

    # Column 0 (mass) should equal [ax, ay, az, 0, 0, 0]
    np.testing.assert_allclose(R[:, 0], [0.5, -0.3, -9.81, 0, 0, 0], atol=1e-12)

    # Inertia columns (4-9) should be all zeros for static case
    np.testing.assert_allclose(R[:, 4:], 0, atol=1e-12)


def test_known_rotation_x():
    """Specific test: omega=[1,0,0] should give Izx=-1 in Ny row (not Iyz)."""
    omega = np.array([1.0, 0, 0])
    alpha = np.zeros(3)
    a_proper = np.zeros(3)

    R = regressor_wrist(a_proper, omega, alpha)

    # Row 4 (Ny): Izx coefficient = wz^2 - wx^2 = 0 - 1 = -1
    assert R[4, 9] == pytest.approx(-1.0), "Izx in Ny should be -1"
    # Iyz should be 0
    assert R[4, 8] == pytest.approx(0.0), "Iyz in Ny should be 0"

    # Row 5 (Nz): Ixy coefficient = wx^2 - wy^2 = 1 - 0 = 1
    assert R[5, 7] == pytest.approx(1.0), "Ixy in Nz should be 1"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
