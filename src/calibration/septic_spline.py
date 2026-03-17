"""
Septic (7th-order) polynomial spline for rest-to-rest joint interpolation.

Uses a normalized basis function h(s) = 35s⁴ - 84s⁵ + 70s⁶ - 20s⁷ that
guarantees zero velocity, acceleration, and jerk at both endpoints.

    q(t) = q_start + (q_end - q_start) * h(t/T)
"""

from __future__ import annotations

import numpy as np


def _h(s: np.ndarray | float) -> np.ndarray | float:
    """Normalized septic basis: h(0)=0, h(1)=1, h'=h''=h'''=0 at endpoints."""
    s2 = s * s
    s4 = s2 * s2
    return 35.0 * s4 - 84.0 * s4 * s + 70.0 * s4 * s2 - 20.0 * s4 * s2 * s


def _dh(s: np.ndarray | float) -> np.ndarray | float:
    """First derivative of h w.r.t. s: dh/ds."""
    s2 = s * s
    s3 = s2 * s
    return 140.0 * s3 - 420.0 * s3 * s + 420.0 * s3 * s2 - 140.0 * s3 * s3


def _ddh(s: np.ndarray | float) -> np.ndarray | float:
    """Second derivative of h w.r.t. s: d²h/ds²."""
    s2 = s * s
    return 420.0 * s2 - 1680.0 * s2 * s + 2100.0 * s2 * s2 - 840.0 * s2 * s2 * s


def septic_position(q0: np.ndarray, q1: np.ndarray, t: float, T: float) -> np.ndarray:
    """Interpolated joint position at time t in [0, T].

    Args:
        q0: Start joint angles (6,).
        q1: End joint angles (6,).
        t: Current time [s].
        T: Total segment duration [s].

    Returns:
        Joint positions (6,).
    """
    s = np.clip(t / T, 0.0, 1.0)
    return q0 + (q1 - q0) * _h(s)


def septic_velocity(q0: np.ndarray, q1: np.ndarray, t: float, T: float) -> np.ndarray:
    """Interpolated joint velocity at time t in [0, T].

    Args:
        q0: Start joint angles (6,).
        q1: End joint angles (6,).
        t: Current time [s].
        T: Total segment duration [s].

    Returns:
        Joint velocities (6,).
    """
    s = np.clip(t / T, 0.0, 1.0)
    return (q1 - q0) * _dh(s) / T


def septic_acceleration(q0: np.ndarray, q1: np.ndarray, t: float, T: float) -> np.ndarray:
    """Interpolated joint acceleration at time t in [0, T].

    Args:
        q0: Start joint angles (6,).
        q1: End joint angles (6,).
        t: Current time [s].
        T: Total segment duration [s].

    Returns:
        Joint accelerations (6,).
    """
    s = np.clip(t / T, 0.0, 1.0)
    return (q1 - q0) * _ddh(s) / (T * T)
