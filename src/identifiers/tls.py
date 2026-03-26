"""
Total Least Squares (TLS) Implementation

This module provides batch TLS solvers with optional weighting matrices.
Based on:
- Golub, G. H., & Van Loan, C. F. (2012). Matrix Computations, §6.3
- Kubus, D., Kröger, T., & Wahl, F. M. (2008). On-line estimation of
  inertial parameters using a recursive total least-squares approach.
- Markovsky, I., & Van Huffel, S. (2007). Overview of total least-squares
  methods. Signal Processing, 87(10), 2283-2302.

Model: (A + E) @ x = b + r
where:
    A: Data matrix (regressor matrix)
    b: Observation vector (wrench)
    x: Parameter vector to estimate (inertial parameters)
    E: Error in data matrix
    r: Error in observation vector

TLS minimizes: ||[E | r]||_F (Frobenius norm)
subject to: (A + E) @ x = b + r

Weighted TLS minimizes: ||D @ [E | r] @ T||_F
where D and T are diagonal weighting matrices.

Scaling modes differ in HOW they construct T:
- IDENTITY: T=I (no scaling)
- DATA_VARIANCE: T=diag(1/std_data) — column normalization for numerical conditioning
- NOISE_VARIANCE: T=diag(1/σ_noise) — ML-optimal weighting under Gaussian noise
"""

import numpy as np
from numpy.typing import NDArray
from typing import Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum


class ScalingMode(Enum):
    """Scaling mode for TLS solver.

    Modes differ in what drives the column weighting T:
    - IDENTITY: No scaling (T=I, D=I). Baseline.
    - DATA_VARIANCE: T=diag(1/std_data). Normalizes columns to unit variance
      for numerical conditioning. Not statistically motivated.
    - NOISE_VARIANCE: T=diag(1/σ_noise). ML-optimal under Gaussian noise.
      Noise std estimated via first differences: σ = std(diff(col)) / √2.

    Legacy aliases NONE, COLUMN_ONLY, FULL are preserved for compatibility.
    """

    IDENTITY = "identity"
    DATA_VARIANCE = "data_variance"
    NOISE_VARIANCE = "noise_variance"


@dataclass
class TLSResult:
    """Result of TLS estimation."""

    x: NDArray[np.floating]  # Parameter estimate (original scale)
    x_scaled: NDArray[np.floating]  # Parameter estimate (scaled space)
    sigma_min: float  # Minimum singular value
    sigma_n: float  # n-th singular value (for condition check)
    condition_ok: bool  # True if σ_n(C1) > σ_{n+1}(C)
    T: NDArray[np.floating]  # Column weighting matrix used
    D: NDArray[np.floating]  # Row weighting matrix used
    info: Dict[str, Any]  # Additional info


def solve_tls_weighted(
    A: NDArray[np.floating],
    b: NDArray[np.floating],
    scaling_mode: ScalingMode = ScalingMode.NOISE_VARIANCE,
    regularization: float = 1e-10,
) -> TLSResult:
    """
    Solve weighted Total Least Squares problem using Golub-Van Loan Algorithm 6.3.1.

    Minimizes: ||D @ [E | r] @ T||_F
    subject to: (A + E) @ x = b + r

    Args:
        A: Data matrix (m x n), m > n
        b: Observation vector (m,)
        scaling_mode: How to construct D and T matrices
        regularization: Small value to prevent division by zero

    Returns:
        TLSResult containing parameter estimate and diagnostic info
    """
    A = np.asarray(A, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64).flatten()

    m, n = A.shape
    if len(b) != m:
        raise ValueError(f"Dimension mismatch: A is {m}x{n}, b has {len(b)} elements")
    if m <= n:
        raise ValueError(f"Need m > n for TLS, got m={m}, n={n}")

    # Form augmented matrix [A | b]
    b_col = b.reshape(-1, 1)
    Aug = np.hstack([A, b_col])  # (m, n+1)

    # Construct weighting matrices D and T based on scaling mode
    D, T = _construct_weighting_matrices(Aug, scaling_mode, regularization)

    # Apply weighting: C = D @ Aug @ T
    C = D @ Aug @ T

    # SVD of weighted augmented matrix
    U, sigma, Vh = np.linalg.svd(C, full_matrices=False)

    # Check TLS solvability condition: σ_n(C1) > σ_{n+1}(C)
    # C1 is the first n columns of C
    C1 = C[:, :n]
    _, sigma_C1, _ = np.linalg.svd(C1, full_matrices=False)
    sigma_n_C1 = sigma_C1[-1] if len(sigma_C1) > 0 else 0.0
    sigma_n_plus_1 = sigma[-1]  # smallest singular value of C

    condition_ok = sigma_n_C1 > sigma_n_plus_1

    if not condition_ok:
        # TLS problem may not have unique solution
        # We still compute a solution but flag the condition
        pass

    # Get V (right singular vectors) - last row of Vh
    v = Vh[-1, :]  # (n+1,)

    # Check for degeneracy
    if np.abs(v[-1]) < regularization:
        raise ValueError(
            "TLS problem is degenerate: last element of v is near zero. "
            "This may indicate rank deficiency or poor data conditioning."
        )

    # Compute TLS solution in SCALED space
    # x_scaled_i = -v_i / v_{n+1}
    x_scaled = -v[:-1] / v[-1]

    # Convert back to ORIGINAL space using T
    # From Golub-Van Loan Eq. (xi = -t_ii * V_{i,n+1} / (t_{n+1,n+1} * V_{n+1,n+1}))
    # Since x_scaled = -v[:-1] / v[-1], and T is diagonal:
    # x_original_i = T_ii * x_scaled_i / T_{n+1,n+1}
    T_diag = np.diag(T)
    x_original = (T_diag[:-1] / T_diag[-1]) * x_scaled

    # Compile result
    result = TLSResult(
        x=x_original,
        x_scaled=x_scaled,
        sigma_min=sigma_n_plus_1,
        sigma_n=sigma_n_C1,
        condition_ok=condition_ok,
        T=T,
        D=D,
        info={
            "scaling_mode": scaling_mode.value,
            "singular_values": sigma.tolist(),
            "m": m,
            "n": n,
        },
    )

    return result


def _construct_weighting_matrices(
    Aug: NDArray[np.floating],
    scaling_mode: ScalingMode,
    regularization: float,
) -> Tuple[NDArray[np.floating], NDArray[np.floating]]:
    """
    Construct D and T weighting matrices based on scaling mode.

    Args:
        Aug: Augmented matrix [A | b], shape (m, n+1)
        scaling_mode: Scaling strategy
        regularization: Small value to prevent division by zero

    Returns:
        Tuple of (D, T) diagonal matrices
    """
    m, n_plus_1 = Aug.shape

    if scaling_mode == ScalingMode.IDENTITY:
        D = np.eye(m)
        T = np.eye(n_plus_1)

    elif scaling_mode == ScalingMode.DATA_VARIANCE:
        # T = diag(1 / std(columns)) — data variance normalization
        col_std = np.std(Aug, axis=0)
        col_std = np.maximum(col_std, regularization)
        T = np.diag(1.0 / col_std)
        D = np.eye(m)

    elif scaling_mode == ScalingMode.NOISE_VARIANCE:
        # T = diag(1 / σ_noise) — ML-optimal under Gaussian noise
        # Estimate noise std via first differences: σ = std(diff(col)) / √2
        # At high sampling rates, diff removes smooth signal, leaving noise.
        noise_std = np.std(np.diff(Aug, axis=0), axis=0) / np.sqrt(2)
        noise_std = np.maximum(noise_std, regularization)
        T = np.diag(1.0 / noise_std)
        D = np.eye(m)

    else:
        raise ValueError(f"Unknown scaling mode: {scaling_mode}")

    return D, T


def solve_tls_batch(
    A: NDArray[np.floating],
    b: NDArray[np.floating],
    scaling_mode: ScalingMode = ScalingMode.NOISE_VARIANCE,
) -> Tuple[NDArray[np.floating], float]:
    """
    Convenience wrapper for solve_tls_weighted.

    This function provides a simpler interface compatible with the existing
    codebase, returning just the parameter estimate and minimum singular value.

    Args:
        A: Data matrix (m x n)
        b: Observation vector (m,)
        scaling_mode: Scaling strategy (default: COLUMN_ONLY)

    Returns:
        Tuple of (parameter estimate, minimum singular value)
    """
    result = solve_tls_weighted(A, b, scaling_mode=scaling_mode)
    return result.x, result.sigma_min


_COMPARE_MODES = [ScalingMode.IDENTITY, ScalingMode.DATA_VARIANCE, ScalingMode.NOISE_VARIANCE]


def solve_tls_compare(
    A: NDArray[np.floating],
    b: NDArray[np.floating],
    modes: list[ScalingMode] | None = None,
) -> Dict[str, TLSResult]:
    """
    Solve TLS with multiple scaling modes and return comparison.

    Args:
        A: Data matrix (m x n)
        b: Observation vector (m,)
        modes: Scaling modes to compare. Defaults to IDENTITY, DATA_VARIANCE,
               NOISE_VARIANCE.

    Returns:
        Dictionary mapping scaling mode name to TLSResult
    """
    if modes is None:
        modes = _COMPARE_MODES

    results = {}
    for mode in modes:
        try:
            result = solve_tls_weighted(A, b, scaling_mode=mode)
            results[mode.value] = result
        except ValueError as e:
            results[mode.value] = None
            print(f"Warning: TLS with {mode.value} scaling failed: {e}")

    return results


def print_tls_comparison(results: Dict[str, TLSResult], param_names: list = None):
    """
    Print formatted comparison of TLS results.

    Args:
        results: Dictionary from solve_tls_compare
        param_names: Optional list of parameter names for display
    """
    if param_names is None:
        param_names = [f"p{i}" for i in range(10)]

    print("\n" + "=" * 70)
    print("TLS Scaling Comparison")
    print("=" * 70)

    # Header
    modes = list(results.keys())
    header = f"{'Param':<8}" + "".join([f"{m:<15}" for m in modes])
    print(header)
    print("-" * 70)

    # Get max length from any result
    max_params = 0
    for r in results.values():
        if r is not None:
            max_params = max(max_params, len(r.x))

    # Print each parameter
    for i in range(max_params):
        name = param_names[i] if i < len(param_names) else f"p{i}"
        row = f"{name:<8}"
        for mode in modes:
            r = results[mode]
            if r is not None and i < len(r.x):
                row += f"{r.x[i]:<15.6f}"
            else:
                row += f"{'N/A':<15}"
        print(row)

    print("-" * 70)

    # Print diagnostics
    print("\nDiagnostics:")
    for mode in modes:
        r = results[mode]
        if r is not None:
            cond_str = "OK" if r.condition_ok else "WARNING"
            print(f"  {mode}: σ_min={r.sigma_min:.4e}, condition={cond_str}")
        else:
            print(f"  {mode}: FAILED")

    print("=" * 70)
