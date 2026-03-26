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
    error_free_cols: list[int] | None = None,
    regularization: float = 1e-10,
) -> TLSResult:
    """
    Solve weighted Total Least Squares problem using Golub-Van Loan Algorithm 6.3.1.

    Minimizes: ||D @ [E | r] @ T||_F
    subject to: (A + E) @ x = b + r

    When error_free_cols is specified, implements Partial EIV (Generalized TLS):
    the indicated columns of A are treated as exact (no perturbation allowed).
    Based on Van Huffel & Vandewalle (1989), Golub & Van Loan §6.3.4 Eq. 6.3.7.

    Args:
        A: Data matrix (m x n), m > n
        b: Observation vector (m,)
        scaling_mode: How to construct D and T matrices
        error_free_cols: Column indices of A that are error-free (e.g. bias columns).
            If specified, these columns are projected out before TLS, then their
            parameters are recovered via OLS on the residual.
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

    if error_free_cols is not None and len(error_free_cols) > 0:
        return _solve_partial_eiv(A, b, scaling_mode, error_free_cols, regularization)

    return _solve_full_tls(A, b, scaling_mode, regularization)


def _solve_full_tls(
    A: NDArray[np.floating],
    b: NDArray[np.floating],
    scaling_mode: ScalingMode,
    regularization: float,
) -> TLSResult:
    """Standard (full) TLS: all columns of A are subject to error."""
    m, n = A.shape

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

    # Get V (right singular vectors) - last row of Vh
    v = Vh[-1, :]  # (n+1,)

    # Check for degeneracy
    if np.abs(v[-1]) < regularization:
        raise ValueError(
            "TLS problem is degenerate: last element of v is near zero. "
            "This may indicate rank deficiency or poor data conditioning."
        )

    # Compute TLS solution in SCALED space
    x_scaled = -v[:-1] / v[-1]

    # Convert back to ORIGINAL space using T
    T_diag = np.diag(T)
    x_original = (T_diag[:-1] / T_diag[-1]) * x_scaled

    return TLSResult(
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


def _solve_partial_eiv(
    A: NDArray[np.floating],
    b: NDArray[np.floating],
    scaling_mode: ScalingMode,
    error_free_cols: list[int],
    regularization: float,
) -> TLSResult:
    """Partial EIV (Generalized TLS): some columns of A are error-free.

    Algorithm:
    1. Split A into A1 (error-free) and A2 (error-prone)
    2. Project out A1's influence: A2' = P_perp @ A2, b' = P_perp @ b
       where P_perp = I - A1 @ A1† (orthogonal complement projector)
    3. Solve standard TLS on [A2' | b']
    4. Recover A1's parameters via OLS: x1 = A1† @ (b - A2 @ x2)

    References:
    - Van Huffel & Vandewalle (1989). Analysis and properties of the
      generalized TLS problem. SIAM J. Matrix Anal. 10:294-315.
    - Golub & Van Loan (2012). Matrix Computations §6.3.4, Eq. 6.3.7.
    """
    m, n = A.shape
    ef_cols = sorted(error_free_cols)
    ep_cols = [j for j in range(n) if j not in ef_cols]

    if len(ep_cols) == 0:
        raise ValueError("All columns are error-free; use OLS instead of TLS.")

    A1 = A[:, ef_cols]  # error-free
    A2 = A[:, ep_cols]  # error-prone

    # Project out A1: P_perp = I - A1 @ A1†
    A1_pinv = np.linalg.pinv(A1)
    P_perp = np.eye(m) - A1 @ A1_pinv

    A2_proj = P_perp @ A2
    b_proj = P_perp @ b

    # Solve reduced TLS on [A2' | b']
    n2 = A2_proj.shape[1]
    if A2_proj.shape[0] <= n2:
        raise ValueError(
            f"Projected system too small for TLS: {A2_proj.shape[0]} rows, {n2} cols"
        )

    reduced_result = _solve_full_tls(A2_proj, b_proj, scaling_mode, regularization)
    x2 = reduced_result.x

    # Recover error-free parameters: x1 = A1† @ (b - A2 @ x2)
    x1 = A1_pinv @ (b - A2 @ x2)

    # Reassemble in original column order
    x_full = np.empty(n, dtype=np.float64)
    x_full[ef_cols] = x1
    x_full[ep_cols] = x2

    return TLSResult(
        x=x_full,
        x_scaled=reduced_result.x_scaled,
        sigma_min=reduced_result.sigma_min,
        sigma_n=reduced_result.sigma_n,
        condition_ok=reduced_result.condition_ok,
        T=reduced_result.T,
        D=reduced_result.D,
        info={
            **reduced_result.info,
            "partial_eiv": True,
            "error_free_cols": ef_cols,
            "error_prone_cols": ep_cols,
            "n_original": n,
            "n_reduced": n2,
        },
    )


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
