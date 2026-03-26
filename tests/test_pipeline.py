"""
Tests for IdentificationPipeline.

Tests cover:
  - IdentificationResult properties (params, bias, mass, param_names)
  - Pipeline process_frame / identify / reset lifecycle
  - Trim window behaviour
  - Difference method (gripper_cal)
  - Error handling (no frames, insufficient frames, bad gripper_cal shape)
  - OLS+bias recovery on synthetic noise-free data
"""

import sys
from pathlib import Path

import numpy as np
import numpy.testing as npt
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from identifiers.pipeline import IdentificationPipeline, IdentificationResult
from identifiers.tls import ScalingMode, TLSResult


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

URDF_PATH = "/root/osx-ur/underlay_ws/src/ur_python_utilities/ur_pykdl/urdf/ur5e.urdf"


@pytest.fixture
def pipeline():
    """Create a pipeline with default settings."""
    return IdentificationPipeline(urdf_path=URDF_PATH)


def _feed_sinusoidal_trajectory(pipeline: IdentificationPipeline, n_frames: int = 200):
    """Feed a sinusoidal joint trajectory into the pipeline.

    Generates smooth joint motion to produce non-trivial kinematics.
    """
    dt = 0.008  # 125 Hz (UR5e default)
    for i in range(n_frames):
        t = i * dt
        phase = 2 * np.pi * t / 1.6  # ~1.6s period
        q = np.array([
            0.1 * np.sin(phase),
            -1.5 + 0.2 * np.sin(phase * 0.7),
            1.0 + 0.15 * np.sin(phase * 1.3),
            -0.5 + 0.1 * np.sin(phase * 0.5),
            0.3 * np.sin(phase * 0.9),
            0.2 * np.sin(phase * 1.1),
        ])
        v = np.array([
            0.1 * 2 * np.pi / 1.6 * np.cos(phase),
            0.2 * 0.7 * 2 * np.pi / 1.6 * np.cos(phase * 0.7),
            0.15 * 1.3 * 2 * np.pi / 1.6 * np.cos(phase * 1.3),
            0.1 * 0.5 * 2 * np.pi / 1.6 * np.cos(phase * 0.5),
            0.3 * 0.9 * 2 * np.pi / 1.6 * np.cos(phase * 0.9),
            0.2 * 1.1 * 2 * np.pi / 1.6 * np.cos(phase * 1.1),
        ])
        wrench = np.random.default_rng(seed=i).standard_normal(6)
        pipeline.process_frame(q, v, t, wrench)


# ---------------------------------------------------------------------------
# IdentificationResult tests
# ---------------------------------------------------------------------------


class TestIdentificationResult:
    """Test IdentificationResult dataclass properties."""

    def _make_result(self) -> IdentificationResult:
        ols_bias = np.array([1.5, 0.01, 0.02, 0.03, 0.1, 0.0, 0.0, 0.1, 0.0, 0.1])
        bias_ols = np.array([0.1, -0.2, 0.3, 0.01, -0.01, 0.005])
        tls_result = TLSResult(
            x=np.zeros(10),
            x_scaled=np.zeros(10),
            sigma_min=0.1,
            sigma_n=1.0,
            condition_ok=True,
            T=np.eye(10),
            D=np.eye(6),
            info={},
        )
        return IdentificationResult(
            ols=np.zeros(10),
            tls=tls_result,
            ols_bias=ols_bias,
            tls_bias=tls_result,
            bias_ols=bias_ols,
            bias_tls=np.zeros(6),
            meta={"total_frames": 100, "trimmed_frames": 90},
        )

    def test_params_returns_ols_bias(self):
        result = self._make_result()
        npt.assert_array_equal(result.params, result.ols_bias)

    def test_bias_returns_bias_ols(self):
        result = self._make_result()
        npt.assert_array_equal(result.bias, result.bias_ols)

    def test_mass(self):
        result = self._make_result()
        assert result.mass == pytest.approx(1.5)

    def test_param_names(self):
        result = self._make_result()
        assert len(result.param_names) == 10
        assert result.param_names[0] == "m"
        assert result.param_names[-1] == "Izz"

    def test_object_params_default_none(self):
        result = self._make_result()
        assert result.object_params is None


# ---------------------------------------------------------------------------
# Pipeline lifecycle tests
# ---------------------------------------------------------------------------


class TestPipelineLifecycle:
    """Test process_frame / identify / reset cycle."""

    def test_initial_frame_count_zero(self, pipeline):
        assert pipeline.frame_count == 0

    def test_process_frame_increments_count(self, pipeline):
        q = np.zeros(6)
        v = np.zeros(6)
        pipeline.process_frame(q, v, 0.0, np.zeros(6))
        assert pipeline.frame_count == 1

    def test_reset_clears_frames(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=20)
        assert pipeline.frame_count == 20
        pipeline.reset()
        assert pipeline.frame_count == 0

    def test_identify_returns_result(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=50)
        result = pipeline.identify()
        assert isinstance(result, IdentificationResult)
        assert result.ols.shape == (10,)
        assert result.ols_bias.shape == (10,)
        assert result.bias_ols.shape == (6,)
        assert result.bias_tls.shape == (6,)
        assert result.tls.x.shape == (10,)
        assert result.tls_bias.x[:10].shape == (10,)

    def test_identify_meta(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=50)
        result = pipeline.identify()
        assert result.meta["total_frames"] == 50
        assert result.meta["trimmed_frames"] == 50
        assert result.meta["tls_scaling"] == "noise_variance"


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------


class TestPipelineErrors:
    """Test error conditions."""

    def test_identify_no_frames_raises(self, pipeline):
        with pytest.raises(ValueError, match="No frames recorded"):
            pipeline.identify()

    def test_identify_insufficient_frames_raises(self, pipeline):
        for i in range(5):
            pipeline.process_frame(np.zeros(6), np.zeros(6), i * 0.01, np.zeros(6))
        with pytest.raises(ValueError, match="Not enough frames"):
            pipeline.identify()

    def test_gripper_cal_wrong_shape_raises(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=50)
        with pytest.raises(ValueError, match="shape"):
            pipeline.identify(gripper_cal=np.zeros(5))


# ---------------------------------------------------------------------------
# Trim window
# ---------------------------------------------------------------------------


class TestTrimWindow:
    """Test trim_start / trim_end behaviour."""

    def test_trim_reduces_frames(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=200)
        # Total duration: 200 * 0.008 = 1.6s
        result = pipeline.identify(trim_start=0.4, trim_end=1.2)
        assert result.meta["total_frames"] == 200
        assert result.meta["trimmed_frames"] < 200
        assert result.meta["trimmed_frames"] > 0

    def test_trim_too_narrow_raises(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=200)
        with pytest.raises(ValueError, match="Not enough frames"):
            pipeline.identify(trim_start=0.0, trim_end=0.01)


# ---------------------------------------------------------------------------
# Difference method
# ---------------------------------------------------------------------------


class TestDifferenceMethod:
    """Test gripper_cal-based difference method."""

    def test_object_params_computed(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=50)
        gripper_cal = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        result = pipeline.identify(gripper_cal=gripper_cal)
        assert result.object_params is not None
        assert set(result.object_params.keys()) == {"OLS", "TLS", "OLS+bias", "TLS+bias"}
        for key, val in result.object_params.items():
            assert val.shape == (10,), f"{key} shape mismatch"

    def test_object_params_is_difference(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=50)
        gripper_cal = np.ones(10) * 0.5
        result = pipeline.identify(gripper_cal=gripper_cal)
        npt.assert_allclose(
            result.object_params["OLS+bias"],
            result.ols_bias - gripper_cal,
        )

    def test_no_gripper_cal_no_object_params(self, pipeline):
        _feed_sinusoidal_trajectory(pipeline, n_frames=50)
        result = pipeline.identify()
        assert result.object_params is None


# ---------------------------------------------------------------------------
# OLS+bias recovery on synthetic data
# ---------------------------------------------------------------------------


class TestOlsBiasRecovery:
    """Verify OLS+bias can recover known parameters on noise-free synthetic data.

    Instead of feeding random wrench, we compute wrench = S @ phi + bias so that
    OLS+bias should perfectly recover phi and bias.
    """

    def test_perfect_recovery(self):
        """OLS+bias recovers exact parameters when wrench = S @ phi + bias."""
        phi_true = np.array([0.5, 0.01, -0.02, 0.03, 0.001, 0.0, 0.0, 0.002, 0.0, 0.001])
        bias_true = np.array([1.0, -0.5, 2.0, 0.1, -0.05, 0.02])

        pipeline = IdentificationPipeline(urdf_path=URDF_PATH)

        n_frames = 300
        dt = 0.008
        for i in range(n_frames):
            t = i * dt
            phase = 2 * np.pi * t / 1.6
            q = np.array([
                0.3 * np.sin(phase),
                -1.5 + 0.4 * np.sin(phase * 0.7),
                1.0 + 0.3 * np.sin(phase * 1.3),
                -0.5 + 0.2 * np.sin(phase * 0.5),
                0.5 * np.sin(phase * 0.9),
                0.4 * np.sin(phase * 1.1),
            ])
            v = np.array([
                0.3 * 2 * np.pi / 1.6 * np.cos(phase),
                0.4 * 0.7 * 2 * np.pi / 1.6 * np.cos(phase * 0.7),
                0.3 * 1.3 * 2 * np.pi / 1.6 * np.cos(phase * 1.3),
                0.2 * 0.5 * 2 * np.pi / 1.6 * np.cos(phase * 0.5),
                0.5 * 0.9 * 2 * np.pi / 1.6 * np.cos(phase * 0.9),
                0.4 * 1.1 * 2 * np.pi / 1.6 * np.cos(phase * 1.1),
            ])

            # Compute kinematics to get the regressor
            result_kin = pipeline._kinematics.compute_with_gravity(q, v, t, pipeline._gravity)
            from utilities.wrist_end_kinematics_utils import get_regressor_matrix
            regressor = get_regressor_matrix(
                linear_acc=result_kin["proper_linear_acceleration"],
                angular_vel=result_kin["angular_velocity"],
                angular_acc=result_kin["angular_acceleration"],
            )

            # Wrench = S @ phi + bias (exact, no noise)
            wrench = regressor @ phi_true + bias_true
            pipeline.process_frame(q, v, t, wrench)

        result = pipeline.identify()

        # OLS+bias should recover phi and bias exactly (up to numerical precision)
        npt.assert_allclose(result.ols_bias, phi_true, atol=1e-6)
        npt.assert_allclose(result.bias_ols, bias_true, atol=1e-6)
        npt.assert_allclose(result.params, phi_true, atol=1e-6)
        npt.assert_allclose(result.bias, bias_true, atol=1e-6)
        assert result.mass == pytest.approx(phi_true[0], abs=1e-6)
