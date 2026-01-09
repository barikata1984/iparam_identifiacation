"""
Utility functions for dynamics calculations, using pymlg for SE3 operations.
Includes the regressor matrix calculation based on Lynch & Park (Modern Robotics) logic.

Requires: pymlg (https://github.com/decargroup/pymlg)
"""

import numpy as np
from pymlg import SE3, SO3
from scipy import constants


def bullet(vec3: np.ndarray) -> np.ndarray:
    """
    Computes the co-adjoint representation related matrix (or similar structure)
    that maps inertial parameters to wrench.

    Equivalent to the 'bullet' operator in the reference implementation.
    Output is (6, 6) but based on the provided dynamics.py implementation,
    it seems to return a specific block structure used for regressor construction.

    In dynamics.py:
    return np.block([
        [x, 0, 0],  # ixx
        [0, y, 0],  # iyy
        [0, 0, z],  # izz
        [y, x, 0],  # ixy
        [0, z, y],  # iyz
        [z, 0, x],  # izx
    ]).T

    This maps [Ixx, Iyy, Izz, Ixy, Iyz, Izx] to the torque components.
    """
    x, y, z = vec3
    return np.block(
        [
            [x, 0, 0],  # ixx
            [0, y, 0],  # iyy
            [0, 0, z],  # izz
            [y, x, 0],  # ixy
            [0, z, y],  # iyz
            [z, 0, x],  # izx
        ]
    ).T


def get_regressor_matrix(twist: np.ndarray, dtwist: np.ndarray) -> np.ndarray:
    """
    Compute the regressor matrix Y(V, dV) such that F = Y @ theta.

    Based on Lynch & Park (Modern Robotics) formulation / dynamics.py implementation.

    Parameters
    ----------
    twist : np.ndarray
        Spatial velocity (twist) in Body Frame. Shape (6,) [v, w] or similar based on definition.
        In dynamics.py, twist is split into v (linear) and w (angular).
        Note: The order in dynamics.py is `v, w = np.split(twist, 2)` -> twist = [v, w].
        However, standard Modern Robotics notation often uses [w, v].
        Based on dynamics.py:
        v, w = np.split(twist, 2) implies twist[:3] is linear, twist[3:] is angular.

    dtwist : np.ndarray
        Spatial acceleration in Body Frame. Shape (6,) [dv, dw].

    Returns
    -------
    np.ndarray
        6x10 Regressor matrix.

        Parameter vector order theta (10x1):
        [m, mc_x, mc_y, mc_z, I_xx, I_yy, I_zz, I_xy, I_yz, I_zx]

        Where:
        - m: Mass
        - mc_*: First moment of mass (mass * center_of_mass)
        - I_*: Inertia tensor components (about the body frame origin)
    """
    # dynamics.py: v, w = np.split(twist, 2)
    # This implies twist is [v; w] (linear, angular)
    v, w = np.split(twist, 2)
    dv, dw = np.split(dtwist, 2)

    # SO3.wedge is the skew-symmetric operator [.]x
    wedge_w = SO3.wedge(w)
    wedge_dw = SO3.wedge(dw)

    bullet_w = bullet(w)
    bullet_dw = bullet(dw)

    # Linear part
    # x = dv + w x v
    x = dv + wedge_w @ v

    wedge_x = SO3.wedge(x)

    # Angular/Coupling terms
    X = wedge_dw + wedge_w @ wedge_w
    Y = bullet_dw + wedge_w @ bullet_w

    # Construct Regressor (6x10)
    # [ x   X   0 ]
    # [ 0  -x^  Y ]
    regressor = np.block(
        [[x.reshape((-1, 1)), X, np.zeros((3, 6))], [np.zeros((3, 1)), -wedge_x, Y]]
    )

    return regressor


def convert_twist_world_to_body(
    twist_world: np.ndarray, rotation_matrix_wb: np.ndarray
) -> np.ndarray:
    """
    Convert a twist from World frame to Body frame.

    Assumptions:
    - world_twist is [v_w; w_w] (Linear; Angular) expressed in World Frame.
    - Reference Point is the Body Origin (at current location).
    - rotation_matrix_wb is R_wb (Rotation from Body to World).

    Formula:
    V_b = [ R_wb^T  0      ]  V_w
          [ 0       R_wb^T ]

    This is valid because the reference point of V_w and V_b is the same (Body Origin),
    only the basis vectors are rotated.

    Parameters
    ----------
    twist_world : (6,) [v, w] in World Frame
    rotation_matrix_wb : (3, 3) R_wb

    Returns
    -------
    twist_body : (6,) [v, w] in Body Frame
    """
    R_bw = rotation_matrix_wb.T

    v_w = twist_world[:3]
    w_w = twist_world[3:]

    v_b = R_bw @ v_w
    w_b = R_bw @ w_w

    return np.concatenate([v_b, w_b])
