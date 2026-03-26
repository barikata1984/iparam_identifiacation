import numpy as np


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
        Columns correspond to: [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]
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
        0,  # Ixy
        0,  # Ixz
        0,  # Iyy
        0,  # Iyz
        0,  # Izz
    ]

    # Row 2 (Fy)
    row2 = [
        ay,  # m
        wx * wy + alz,  # hx
        -wx2 - wz2,  # hy
        wy * wz - alx,  # hz
        0,  # Ixx
        0,  # Ixy
        0,  # Ixz
        0,  # Iyy
        0,  # Iyz
        0,  # Izz
    ]

    # Row 3 (Fz)
    row3 = [
        az,  # m
        wx * wz - aly,  # hx
        wy * wz + alx,  # hy
        -wx2 - wy2,  # hz
        0,  # Ixx
        0,  # Ixy
        0,  # Ixz
        0,  # Iyy
        0,  # Iyz
        0,  # Izz
    ]

    # Row 4 (Nx)
    row4 = [
        0,  # m
        0,  # hx
        az,  # hy
        -ay,  # hz
        alx,  # Ixx
        aly - wx * wz,  # Ixy
        alz + wx * wy,  # Ixz
        -wy * wz,  # Iyy
        wy2 - wz2,  # Iyz
        wy * wz,  # Izz
    ]

    # Row 5 (Ny)
    row5 = [
        0,  # m
        -az,  # hx
        0,  # hy
        ax,  # hz
        wx * wz,  # Ixx
        alx + wy * wz,  # Ixy
        wz2 - wx2,  # Ixz
        aly,  # Iyy
        alz - wx * wy,  # Iyz
        -wx * wz,  # Izz
    ]

    # Row 6 (Nz)
    row6 = [
        0,  # m
        ay,  # hx
        -ax,  # hy
        0,  # hz
        -wx * wy,  # Ixx
        wx2 - wy2,  # Ixy
        alx - wy * wz,  # Ixz
        wx * wy,  # Iyy
        aly + wx * wz,  # Iyz
        alz,  # Izz
    ]

    return np.array([row1, row2, row3, row4, row5, row6])
