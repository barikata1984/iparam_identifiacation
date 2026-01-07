#!/usr/bin/env python3
import numpy as np


class NumericalDifferentiator:
    """
    Simple numerical differentiator with low-pass filtering.
    """

    def __init__(self, cutoff_freq: float = 10.0):
        """
        Parameters
        ----------
        cutoff_freq : float
            Cutoff frequency for low-pass filter (Hz)
        """
        self.prev_value = None
        self.prev_time = None
        self.prev_derivative = None
        self.cutoff_freq = cutoff_freq

    def update(self, value: np.ndarray, time: float) -> np.ndarray:
        """
        Compute the derivative of the input signal.

        Parameters
        ----------
        value : np.ndarray
            Current value
        time : float
            Current timestamp

        Returns
        -------
        np.ndarray
            Filtered derivative
        """
        if self.prev_value is None:
            self.prev_value = value
            self.prev_time = time
            self.prev_derivative = np.zeros_like(value)
            return self.prev_derivative

        dt = time - self.prev_time
        if dt <= 0:
            return self.prev_derivative

        # Raw derivative
        raw_derivative = (value - self.prev_value) / dt

        # Low-pass filter (first-order)
        alpha = dt * self.cutoff_freq / (1 + dt * self.cutoff_freq)
        filtered_derivative = (
            alpha * raw_derivative + (1 - alpha) * self.prev_derivative
        )

        # Update state
        self.prev_value = value
        self.prev_time = time
        self.prev_derivative = filtered_derivative

        return filtered_derivative

    def reset(self):
        """Reset the differentiator state."""
        self.prev_value = None
        self.prev_time = None
        self.prev_derivative = None
