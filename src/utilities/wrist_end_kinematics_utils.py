import numpy as np
from pymlg import SE3, SO3


def coordinate_transform_linang_velacc(rot, vel, acc, gravity):
    """
    Computes kinematics (lv, av, la, aa) in tool0 frame.

    Parameters
    ----------
    rot_tool0_base : np.ndarray
        3x3 rotation matrix from base to tool0
    vel_base_tool0 : np.ndarray
        6D velocity in base frame [vx, vy, vz, wx, wy, wz]
    acc_base_tool0 : np.ndarray
        6D acceleration in base frame [ax, ay, az, alx, aly, alz]
    gravity : np.ndarray
        Gravity vector in base frame e.g. [0, 0, -9.81]

    Returns
    -------
    tuple
        (lv_tool0, av_tool0, la_tool0, aa_tool0)
        All are 3D vectors (np.ndarray).
    """
    from pymlg import SO3

    # Coordinate-transform the velocities from base to tool0
    lv = vel[:3]
    av = vel[3:]

    lv_new = rot @ lv
    av_new = rot @ av

    # Coordinate-transform the accelecations from base to tool0
    _la = acc[:3]
    aa = acc[3:]

    # Proper acceleration: a_proper = a_kinematic - g
    la = _la - gravity

    # Angular acceleration: R^T * alpha_s
    aa_new = rot @ aa

    # Linear acceleration: R^T * a_proper - w_b x v_b (Coriolis/Convective term)
    coriolis_term = rot @ SO3.wedge(av) @ lv
    la_new = rot @ la - coriolis_term

    return lv_new, av_new, la_new, aa_new


def get_pose(pose_list, inverse=False, only_rot=False):
    """
    Converts a pose list [x, y, z, qx, qy, qz, qw] to a 4x4 homogenous matrix (SE3).

    Parameters
    ----------
    pose_list : list or array-like
        [x, y, z, qx, qy, qz, qw]
    inverse : bool, optional
        If True, returns the inverse of the matrix, by default False
    only_rot : bool, optional
        If True, returns only the 3x3 rotation matrix component, by default False

    Returns
    -------
    np.ndarray
        4x4 homogeneous matrix or 3x3 rotation matrix.
    """
    pos = np.array(pose_list[:3])
    quat = np.array(pose_list[3:])

    # Let's use SO3 from pymlg as before.

    rot = SO3.from_quat(quat, order="xyzw")

    T = SE3.from_components(rot, pos)

    if inverse:
        T_inv = np.linalg.inv(T)
        if only_rot:
            return T_inv[:3, :3]
        return T_inv

    if only_rot:
        return T[:3, :3]

    return T


def get_regressor_matrix(linear_acc, angular_vel, angular_acc):
    """
    Computes the 6x10 regressor matrix S_A relating inertial parameters to wrenches.
    W = S_A * pi

    Parameters
    ----------
    linear_acc : array-like (3,)
        Linear acceleration (proper acceleration) [ax, ay, az]
    angular_vel : array-like (3,)
        Angular velocity [wx, wy, wz]
    angular_acc : array-like (3,)
        Angular acceleration [alx, aly, alz]

    Returns
    -------
    np.ndarray
        6x10 regressor matrix.
        Columns correspond to: [m, hx, hy, hz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    """
    ax, ay, az = linear_acc
    wx, wy, wz = angular_vel
    alx, aly, alz = angular_acc

    wx2 = wx * wx
    wy2 = wy * wy
    wz2 = wz * wz

    # Row 1 (Fx)
    row1 = [
        ax,  # m
        -wy2 - wz2,  # hx
        wx * wy - alz,  # hy
        wx * wz + aly,  # hz
        0,  # Ixx
        0,  # Iyy
        0,  # Izz
        0,  # Ixy
        0,  # Ixz
        0,  # Iyz
    ]

    # Row 2 (Fy)
    row2 = [
        ay,  # m
        wx * wy + alz,  # hx
        -wx2 - wz2,  # hy
        wy * wz - alx,  # hz
        0,  # Ixx
        0,  # Iyy
        0,  # Izz
        0,  # Ixy
        0,  # Ixz
        0,  # Iyz
    ]

    # Row 3 (Fz)
    row3 = [
        az,  # m
        wx * wz - aly,  # hx
        wy * wz + alx,  # hy
        -wx2 - wy2,  # hz
        0,  # Ixx
        0,  # Iyy
        0,  # Izz
        0,  # Ixy
        0,  # Ixz
        0,  # Iyz
    ]

    # Row 4 (Nx)
    row4 = [
        0,  # m
        0,  # hx
        az,  # hy
        -ay,  # hz
        alx,  # Ixx
        -wy * wz,  # Iyy
        wy * wz,  # Izz
        aly - wx * wz,  # Ixy
        alz + wx * wy,  # Ixz
        wy2 - wz2,  # Iyz
    ]

    # Row 5 (Ny)
    row5 = [
        0,  # m
        -az,  # hx
        0,  # hy
        ax,  # hz
        wx * wz,  # Ixx
        alx + wy * wz,  # Iyy
        -wx * wz,  # Izz
        aly,  # Ixy
        alz - wx * wy,  # Ixz
        wz2 - wx2,  # Iyz
    ]

    # Row 6 (Nz)
    row6 = [
        0,  # m
        ay,  # hx
        -ax,  # hy
        0,  # hz
        -wx * wy,  # Ixx
        wx * wy,  # Iyy
        alz,  # Izz
        alx - wy * wz,  # Ixy
        aly + wx * wz,  # Ixz
        wz2 - wy2,  # Iyz
    ]

    return np.array([row1, row2, row3, row4, row5, row6])
