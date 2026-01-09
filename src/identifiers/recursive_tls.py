"""
Recursive Total Least Squares (RTLS) Implementation

This module provides a recursive implementation of total least squares
for online parameter estimation. Unlike OLS, TLS considers noise in both
the input (regressor matrix) and output measurements.

Model: (A + E_A) @ x = y + e_y
where:
    y: output vector (noisy measurements)
    A: regressor matrix (noisy data matrix)
    x: parameter vector to be estimated
    E_A: noise in the regressor matrix
    e_y: noise in the output

TLS minimizes: ||[E_A, e_y]||_F (Frobenius norm)
subject to: (A + E_A) @ x = y + e_y

Reference:
    Kubus, D., Kröger, T., & Wahl, F. M. (2008).
    "On-line estimation of inertial parameters using a recursive total least-squares approach."
    IEEE/RSJ International Conference on Intelligent Robots and Systems.
"""

import numpy as np
from numpy.typing import NDArray
from typing import Tuple, Optional


class RecursiveTLS:
    """
    Recursive Total Least Squares (RTLS) estimator.

    This implementation uses the recursive update of the augmented data matrix
    covariance to handle noise in both input and output data.

    Attributes:
        n_params: Number of parameters to estimate
        forgetting_factor: Forgetting factor (0 < λ <= 1) for exponential weighting
        C: Augmented covariance matrix of [A, y]
        x: Current parameter estimate
    """

    def __init__(
        self,
        n_params: int,
        forgetting_factor: float = 1.0,
        regularization: float = 1e-8,
        deadzone: float = 0.1,
    ):
        """
        Initialize the RTLS estimator.

        Args:
            n_params: Number of parameters to estimate
            forgetting_factor: Forgetting factor λ (0 < λ <= 1).
                              λ = 1 gives equal weight to all data.
                              λ < 1 gives more weight to recent data.
            regularization: Small value for numerical stability
        """
        if not 0 < forgetting_factor <= 1:
            raise ValueError("Forgetting factor must be in (0, 1]")

        self.n_params = n_params
        self.forgetting_factor = forgetting_factor
        self.forgetting_factor = forgetting_factor
        self.regularization = regularization
        self.deadzone = deadzone

        # Dimension of augmented vector [a, y]
        self.aug_dim = n_params + 1

        # Initialize augmented covariance matrix C = E[[a; y] @ [a; y].T]
        # Start with small values (will be built up from data)
        self.C = regularization * np.eye(self.aug_dim)

        # Initialize parameter estimate
        self.x = np.zeros(n_params)

        # Track number of updates
        self.n_updates = 0

        # Store sum of weights for normalization
        self.weight_sum = 0.0

    def _solve_tls_from_covariance(self) -> NDArray[np.floating]:
        """
        Solve TLS problem from the current covariance matrix.

        The TLS solution is the eigenvector corresponding to the smallest
        eigenvalue of the augmented covariance matrix C.

        Returns:
            Parameter estimate x
        """
        # Eigenvalue decomposition of C
        eigenvalues, eigenvectors = np.linalg.eigh(self.C)

        # Find the eigenvector corresponding to the smallest eigenvalue
        min_idx = np.argmin(eigenvalues)
        v = eigenvectors[:, min_idx]

        # The eigenvector is [x; -1] (up to scaling)
        # Normalize so that the last element is -1
        if np.abs(v[-1]) < 1e-10:
            # Degenerate case: use pseudo-inverse approach
            return self.x

        x = -v[:-1] / v[-1]

        return x

    def update(self, a: NDArray[np.floating], y: float) -> NDArray[np.floating]:
        """
        Perform a single recursive update with a new measurement.

        Args:
            a: Regressor vector (1D array of shape (n_params,))
            y: Scalar measurement

        Returns:
            Updated parameter estimate
        """
        a = np.asarray(a).flatten()
        if len(a) != self.n_params:
            raise ValueError(f"Regressor vector must have {self.n_params} elements")

        # Conditional Update: Check for sufficient excitation
        # If the regressor vector is too small (e.g. robot is not moving),
        # skip the update to prevent covariance decay (TLS) or wind-up (OLS).
        if np.linalg.norm(a) < self.deadzone:
            return self.x.copy()

        λ = self.forgetting_factor

        # Form augmented vector z = [a; y]
        z = np.append(a, y)

        # Update covariance matrix with forgetting factor
        # C = λ * C + z @ z.T
        self.C = λ * self.C + np.outer(z, z)

        # Update weight sum for proper normalization
        self.weight_sum = λ * self.weight_sum + 1.0

        # Solve TLS problem from updated covariance
        self.x = self._solve_tls_from_covariance()

        self.n_updates += 1

        return self.x.copy()

    def update_batch(
        self,
        A: NDArray[np.floating],
        y: NDArray[np.floating],
    ) -> NDArray[np.floating]:
        """
        Perform batch recursive updates with multiple measurements.

        Args:
            A: Regressor matrix (n_samples, n_params)
            y: Measurement vector (n_samples,)

        Returns:
            Updated parameter estimate
        """
        A = np.asarray(A)
        y = np.asarray(y).flatten()

        if A.ndim == 1:
            A = A.reshape(1, -1)

        if len(y) != A.shape[0]:
            raise ValueError("Number of measurements must match number of rows in A")

        for i in range(len(y)):
            self.update(A[i], y[i])

        return self.x.copy()

    def get_estimate(self) -> NDArray[np.floating]:
        """Return the current parameter estimate."""
        return self.x.copy()

    def get_covariance(self) -> NDArray[np.floating]:
        """Return the current augmented covariance matrix."""
        return self.C.copy()

    def get_normalized_covariance(self) -> NDArray[np.floating]:
        """Return the normalized covariance matrix."""
        if self.weight_sum > 0:
            return self.C / self.weight_sum
        return self.C.copy()

    def reset(self) -> None:
        """Reset the estimator to initial state."""
        self.C = self.regularization * np.eye(self.aug_dim)
        self.x = np.zeros(self.n_params)
        self.n_updates = 0
        self.weight_sum = 0.0


def solve_tls_batch(
    A: NDArray[np.floating],
    y: NDArray[np.floating],
) -> Tuple[NDArray[np.floating], float]:
    """
    Solve total least squares problem in batch mode using SVD.

    Solves: min_{E_A, e_y} ||[E_A, e_y]||_F
    subject to: (A + E_A) @ x = y + e_y

    Args:
        A: Regressor matrix (n_samples, n_params)
        y: Measurement vector (n_samples,)

    Returns:
        Tuple of (parameter estimate x, smallest singular value)
    """
    A = np.asarray(A)
    y = np.asarray(y).flatten().reshape(-1, 1)

    # Form augmented matrix [A, y]
    B = np.hstack([A, y])

    # SVD of augmented matrix
    U, s, Vh = np.linalg.svd(B, full_matrices=False)

    # The TLS solution is derived from the last row of V (or last column of Vh)
    v = Vh[-1, :]

    # Normalize so that the last element is -1
    if np.abs(v[-1]) < 1e-10:
        raise ValueError("TLS problem is degenerate (last element of v is zero)")

    x = -v[:-1] / v[-1]

    return x, s[-1]


def solve_tls_truncated(
    A: NDArray[np.floating],
    y: NDArray[np.floating],
    rank: Optional[int] = None,
) -> NDArray[np.floating]:
    """
    Solve truncated total least squares problem.

    This is useful when the regressor matrix is rank-deficient or
    when dimensionality reduction is desired.

    Args:
        A: Regressor matrix (n_samples, n_params)
        y: Measurement vector (n_samples,)
        rank: Desired rank for truncation (default: n_params - 1)

    Returns:
        Parameter estimate x
    """
    A = np.asarray(A)
    y = np.asarray(y).flatten().reshape(-1, 1)

    n_params = A.shape[1]
    if rank is None:
        rank = n_params

    # Form augmented matrix [A, y]
    B = np.hstack([A, y])

    # SVD of augmented matrix
    U, s, Vh = np.linalg.svd(B, full_matrices=False)

    # Truncate to desired rank
    U_k = U[:, :rank]
    s_k = s[:rank]
    Vh_k = Vh[:rank, :]

    # Reconstruct truncated matrix
    B_k = U_k @ np.diag(s_k) @ Vh_k

    # Split back into A and y
    A_k = B_k[:, :-1]
    y_k = B_k[:, -1]

    # Solve OLS on the truncated problem
    x, _, _, _ = np.linalg.lstsq(A_k, y_k, rcond=None)

    return x
