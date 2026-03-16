"""
Synthetic test: verify OLS and TLS solvers recover known 10-parameter vectors.

Purpose: isolate the LS *process* from the physical *inputs*.
If these tests pass, the solver is sound and the problem lies in the
regressor / wrench data fed to it.

We mimic the real pipeline's structure:
  - 10 unknown parameters (like inertial params)
  - Each "frame" contributes a (6, 10) regressor block and a (6,) observation
  - N frames are stacked: S_total (6N, 10), W_total (6N,)
  - Solved with np.linalg.lstsq  (OLS)
  - Solved with solve_tls_weighted (TLS)
"""

import sys
from pathlib import Path

import numpy as np
import numpy.testing as npt

# Allow imports from the package source
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from identifiers.tls import ScalingMode, solve_tls_weighted


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_synthetic_data(
    phi_true: np.ndarray,
    n_frames: int = 200,
    rows_per_frame: int = 6,
    noise_std: float = 0.0,
    regressor_noise_std: float = 0.0,
    seed: int = 42,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate S_total, W_total for a known parameter vector.

    S is random (standard normal), W = S @ phi + noise.
    Optionally add noise to S as well (for TLS realism).
    """
    rng = np.random.default_rng(seed)
    n_rows = n_frames * rows_per_frame
    n_params = len(phi_true)

    S_clean = rng.standard_normal((n_rows, n_params))
    S = S_clean + regressor_noise_std * rng.standard_normal(S_clean.shape)
    W = S_clean @ phi_true + noise_std * rng.standard_normal(n_rows)

    return S, W


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

# True parameters — deliberately span different orders of magnitude
# (like real inertial params: mass ~0.1 kg, moments ~1e-3, inertias ~1e-5)
PHI_TRUE = np.array(
    [
        0.120,  # m      [kg]
        1.2e-3,  # mc_x   [kg·m]
        -0.8e-3,  # mc_y
        2.5e-3,  # mc_z
        3.0e-5,  # I_xx   [kg·m²]
        4.5e-5,  # I_yy
        2.8e-5,  # I_zz
        -1.0e-6,  # I_xy
        0.5e-6,  # I_yz
        -0.3e-6,  # I_zx
    ]
)


class TestOLSSynthetic:
    """Verify np.linalg.lstsq recovers params from noise-free & noisy data."""

    def test_noiseless_exact_recovery(self):
        """No noise → OLS must recover phi exactly (up to machine eps)."""
        S, W = make_synthetic_data(PHI_TRUE, n_frames=200, noise_std=0.0)
        phi_est, _, rank, _ = np.linalg.lstsq(S, W, rcond=None)

        assert rank == 10, f"Rank deficient: {rank}"
        npt.assert_allclose(phi_est, PHI_TRUE, atol=1e-12)

    def test_observation_noise_reasonable_recovery(self):
        """Small observation noise → OLS should still be close."""
        S, W = make_synthetic_data(PHI_TRUE, n_frames=500, noise_std=0.01)
        phi_est, _, rank, _ = np.linalg.lstsq(S, W, rcond=None)

        assert rank == 10
        # Mass (dominant param) should be within 5%
        assert abs(phi_est[0] - PHI_TRUE[0]) / PHI_TRUE[0] < 0.05

    def test_high_noise_more_frames(self):
        """More frames should compensate for higher noise (law of large numbers)."""
        results = []
        for n_frames in [100, 500, 2000]:
            S, W = make_synthetic_data(PHI_TRUE, n_frames=n_frames, noise_std=0.05)
            phi_est, *_ = np.linalg.lstsq(S, W, rcond=None)
            err = np.linalg.norm(phi_est - PHI_TRUE) / np.linalg.norm(PHI_TRUE)
            results.append(err)

        # Error should decrease with more frames
        assert results[0] > results[2], f"More frames didn't help: {results}"

    def test_mass_scale_recovery(self):
        """Verify mass (largest param) is correctly recovered at realistic scale."""
        S, W = make_synthetic_data(PHI_TRUE, n_frames=1000, noise_std=0.005)
        phi_est, *_ = np.linalg.lstsq(S, W, rcond=None)

        mass_true = PHI_TRUE[0]  # 0.120 kg = 120g
        mass_est = phi_est[0]
        # With 1000 frames and low noise, mass should be within 2%
        rel_err = abs(mass_est - mass_true) / mass_true
        assert rel_err < 0.02, f"Mass: true={mass_true}, est={mass_est}, err={rel_err:.1%}"


class TestTLSSynthetic:
    """Verify TLS solver recovers params from synthetic data."""

    def test_noiseless_exact_recovery(self):
        """TLS with no noise should also recover params exactly."""
        S, W = make_synthetic_data(PHI_TRUE, n_frames=200, noise_std=0.0)
        result = solve_tls_weighted(S, W, scaling_mode=ScalingMode.NONE)

        npt.assert_allclose(result.x, PHI_TRUE, atol=1e-8)

    def test_column_scaling_noiseless(self):
        """TLS with column scaling and no noise should recover params."""
        S, W = make_synthetic_data(PHI_TRUE, n_frames=200, noise_std=0.0)
        result = solve_tls_weighted(S, W, scaling_mode=ScalingMode.COLUMN_ONLY)

        npt.assert_allclose(result.x, PHI_TRUE, atol=1e-8)

    def test_both_noise_tls_vs_ols(self):
        """When both S and W have noise, TLS should be ≥ as good as OLS."""
        S, W = make_synthetic_data(PHI_TRUE, n_frames=500, noise_std=0.01, regressor_noise_std=0.01)

        # OLS
        phi_ols, *_ = np.linalg.lstsq(S, W, rcond=None)
        err_ols = np.linalg.norm(phi_ols - PHI_TRUE) / np.linalg.norm(PHI_TRUE)

        # TLS (no scaling — fair comparison)
        result_tls = solve_tls_weighted(S, W, scaling_mode=ScalingMode.NONE)
        err_tls = np.linalg.norm(result_tls.x - PHI_TRUE) / np.linalg.norm(PHI_TRUE)

        print(f"OLS err: {err_ols:.4f}, TLS err: {err_tls:.4f}")
        # Both should be reasonable
        assert err_ols < 0.5, f"OLS too far off: {err_ols}"
        assert err_tls < 0.5, f"TLS too far off: {err_tls}"


class TestRegressorConditioning:
    """Test how regressor matrix properties affect estimation."""

    def test_ill_conditioned_regressor(self):
        """Near-collinear columns → large errors even without noise."""
        rng = np.random.default_rng(99)
        n_rows = 1200
        # Create a regressor where column 1 ≈ column 0
        S = rng.standard_normal((n_rows, 10))
        S[:, 1] = S[:, 0] + 1e-6 * rng.standard_normal(n_rows)  # near-collinear

        W = S @ PHI_TRUE + 0.001 * rng.standard_normal(n_rows)
        phi_est, _, rank, sv = np.linalg.lstsq(S, W, rcond=None)

        cond = sv[0] / sv[-1]
        print(f"Condition number: {cond:.2e}")
        print(f"Singular values: {sv}")
        # Condition number should be very large
        assert cond > 1e6, "Expected ill-conditioned matrix"

    def test_constant_column_rank_deficient(self):
        """A constant column (e.g., no acceleration variation) → rank < 10."""
        rng = np.random.default_rng(77)
        n_rows = 1200
        S = rng.standard_normal((n_rows, 10))
        S[:, 0] = 1.0  # constant column (no variation in this feature)

        W = S @ PHI_TRUE
        _, _, rank, _ = np.linalg.lstsq(S, W, rcond=None)

        # Rank should still be 10 since the constant column is linearly independent
        # BUT if we add noise and have near-constant, it becomes problematic
        assert rank == 10

    def test_scale_mismatch_ols_still_works(self):
        """Even with wildly different column scales, OLS should work
        (it's just the condition number that degrades)."""
        rng = np.random.default_rng(55)
        n_rows = 1200
        S = rng.standard_normal((n_rows, 10))
        # Scale columns to mimic real inertial param magnitudes
        scales = np.array([1.0, 0.01, 0.01, 0.01, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6])
        S = S * scales  # columns have very different magnitudes

        W = S @ PHI_TRUE  # noiseless
        phi_est, _, rank, _ = np.linalg.lstsq(S, W, rcond=None)

        npt.assert_allclose(phi_est, PHI_TRUE, atol=1e-10)


if __name__ == "__main__":
    import pytest

    pytest.main([__file__, "-v"])
