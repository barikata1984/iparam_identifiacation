"""
IdentificationPipeline — ROS-independent inertial parameter estimation.

Wraps Tool0KinematicsCalculator, get_regressor_matrix, and 4 estimation methods
(OLS, TLS, OLS+bias, TLS+bias) into a single pipeline class.

Extracted from ExcitationTrajectoryReplayNode._recording_callback() + phase_identify().

Usage:
    pipeline = IdentificationPipeline()
    for q, v, t, wrench in recorded_data:
        pipeline.process_frame(q, v, t, wrench)
    result = pipeline.identify()
    print(result.params)  # OLS+bias (recommended)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from identifiers.tls import ScalingMode, TLSResult, solve_tls_weighted
from utilities.identification_utils import PARAM_NAMES
from utilities.tool0_kinematics import DEFAULT_URDF_PATH, Tool0KinematicsCalculator
from utilities.wrist_end_kinematics_utils import get_regressor_matrix


@dataclass
class IdentificationResult:
    """Result of inertial parameter identification with 4 methods.

    Attributes:
        ols: OLS estimate (10,).
        tls: TLS result (TLSResult.x is (10,)).
        ols_bias: OLS+bias estimate (10,) — recommended method.
        tls_bias: TLS+bias result (TLSResult.x[:10] is (10,)).
        bias_ols: F/T bias from OLS+bias (6,).
        bias_tls: F/T bias from TLS+bias (6,).
        object_params: Per-method object inertia via difference method, or None.
        meta: Diagnostics (total_frames, trimmed_frames, tls_scaling, etc.).
    """

    ols: NDArray[np.floating]
    tls: TLSResult
    ols_bias: NDArray[np.floating]
    tls_bias: TLSResult
    bias_ols: NDArray[np.floating]
    bias_tls: NDArray[np.floating]
    object_params: dict[str, NDArray[np.floating]] | None = None
    meta: dict = field(default_factory=dict)

    @property
    def params(self) -> NDArray[np.floating]:
        """Recommended inertial parameters (OLS+bias)."""
        return self.ols_bias

    @property
    def bias(self) -> NDArray[np.floating]:
        """Recommended F/T bias estimate (OLS+bias)."""
        return self.bias_ols

    @property
    def mass(self) -> float:
        """Estimated mass from recommended method [kg]."""
        return float(self.params[0])

    @property
    def param_names(self) -> list[str]:
        """Parameter names: [m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]."""
        return list(PARAM_NAMES)


@dataclass
class _Frame:
    """Internal storage for a single recorded frame."""

    time: float
    regressor: NDArray[np.floating]  # (6, 10)
    wrench: NDArray[np.floating]  # (6,)


class IdentificationPipeline:
    """ROS-independent pipeline for inertial parameter estimation.

    Accumulates frames via process_frame(), then estimates inertial parameters
    via identify(). All kinematics and regressor computation is internal.

    Args:
        urdf_path: Path to UR5e URDF for kinematics.
        tls_scaling: Scaling mode for TLS solver.
        acc_cutoff_freq: Cutoff frequency for acceleration low-pass filter [Hz].
        gravity: Gravity vector in base frame [m/s²].
    """

    def __init__(
        self,
        urdf_path: str = DEFAULT_URDF_PATH,
        tls_scaling: ScalingMode = ScalingMode.NOISE_VARIANCE,
        acc_cutoff_freq: float = 10.0,
        gravity: NDArray[np.floating] | None = None,
    ):
        self._tls_scaling = tls_scaling
        self._gravity = (
            np.array(gravity) if gravity is not None else np.array([0.0, 0.0, -9.81])
        )
        self._kinematics = Tool0KinematicsCalculator(
            urdf_path=urdf_path, acc_cutoff_freq=acc_cutoff_freq
        )
        self._frames: list[_Frame] = []

    def process_frame(
        self,
        q: NDArray[np.floating],
        v: NDArray[np.floating],
        t: float,
        wrench: NDArray[np.floating],
    ) -> None:
        """Accumulate one frame: compute kinematics, build regressor, store.

        Args:
            q: Joint positions (6,) in pinocchio order.
            v: Joint velocities (6,) in pinocchio order.
            t: Timestamp [seconds].
            wrench: F/T sensor reading (6,) [Fx, Fy, Fz, Tx, Ty, Tz].
        """
        result = self._kinematics.compute_with_gravity(
            np.asarray(q, dtype=np.float64),
            np.asarray(v, dtype=np.float64),
            t,
            self._gravity,
        )
        regressor = get_regressor_matrix(
            linear_acc=result["proper_linear_acceleration"],
            angular_vel=result["angular_velocity"],
            angular_acc=result["angular_acceleration"],
        )
        self._frames.append(
            _Frame(time=t, regressor=regressor, wrench=np.asarray(wrench, dtype=np.float64))
        )

    def identify(
        self,
        trim_start: float = 0.0,
        trim_end: float = math.inf,
        gripper_cal: NDArray[np.floating] | None = None,
    ) -> IdentificationResult:
        """Estimate inertial parameters from accumulated frames.

        Args:
            trim_start: Start of analysis window [seconds from first frame].
            trim_end: End of analysis window [seconds from first frame].
            gripper_cal: Gripper-only inertial parameters (10,) for difference method.
                When provided, object_params is computed as φ_total − φ_gripper
                for each method.

        Returns:
            IdentificationResult with all 4 methods.

        Raises:
            ValueError: If fewer than 10 frames in the trim window.
        """
        if len(self._frames) == 0:
            raise ValueError("No frames recorded. Call process_frame() first.")

        # Trim to analysis window (relative time from first frame)
        t0 = self._frames[0].time
        trimmed = [
            f for f in self._frames if trim_start <= (f.time - t0) <= trim_end
        ]
        if len(trimmed) < 10:
            raise ValueError(
                f"Not enough frames in trim window: {len(trimmed)} < 10 "
                f"(total: {len(self._frames)}, window: [{trim_start}, {trim_end}])"
            )

        N = len(trimmed)
        S_total = np.vstack([f.regressor for f in trimmed])  # (6*N, 10)
        W_total = np.hstack([f.wrench for f in trimmed])  # (6*N,)

        # Bias augmentation: [S | I₆] @ [φ; b] = W (Kubus et al. 2007 Approach 2)
        bias_block = np.tile(np.eye(6), (N, 1))  # (6*N, 6)
        S_aug = np.hstack([S_total, bias_block])  # (6*N, 16)

        # --- OLS ---
        pi_ols, _, _, _ = np.linalg.lstsq(S_total, W_total, rcond=None)

        # --- TLS ---
        tls_result = solve_tls_weighted(
            S_total, W_total, scaling_mode=self._tls_scaling
        )

        # --- OLS+bias ---
        pi_aug, _, _, _ = np.linalg.lstsq(S_aug, W_total, rcond=None)
        ols_bias_params = pi_aug[:10]
        bias_ols = pi_aug[10:]

        # --- TLS+bias (Partial EIV: bias columns are error-free) ---
        bias_col_indices = list(range(10, 16))
        tls_bias_result = solve_tls_weighted(
            S_aug,
            W_total,
            scaling_mode=self._tls_scaling,
            error_free_cols=bias_col_indices,
        )
        bias_tls = tls_bias_result.x[10:]

        # --- Difference method ---
        object_params = None
        if gripper_cal is not None:
            gripper_cal = np.asarray(gripper_cal, dtype=np.float64)
            if gripper_cal.shape != (10,):
                raise ValueError(
                    f"gripper_cal must be shape (10,), got {gripper_cal.shape}"
                )
            object_params = {
                "OLS": pi_ols - gripper_cal,
                "TLS": tls_result.x - gripper_cal,
                "OLS+bias": ols_bias_params - gripper_cal,
                "TLS+bias": tls_bias_result.x[:10] - gripper_cal,
            }

        return IdentificationResult(
            ols=pi_ols,
            tls=tls_result,
            ols_bias=ols_bias_params,
            tls_bias=tls_bias_result,
            bias_ols=bias_ols,
            bias_tls=bias_tls,
            object_params=object_params,
            meta={
                "total_frames": len(self._frames),
                "trimmed_frames": N,
                "trim_start": trim_start,
                "trim_end": trim_end,
                "tls_scaling": self._tls_scaling.value,
            },
        )

    def reset(self) -> None:
        """Clear accumulated frames and reset kinematics state."""
        self._frames.clear()
        self._kinematics.reset()

    @property
    def frame_count(self) -> int:
        """Number of accumulated frames."""
        return len(self._frames)
