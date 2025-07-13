from typing import Optional, Union

import numpy as np


class PIDController:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        dt: float,
        min_output: Optional[float] = None,
        max_output: Optional[float] = None,
        feed_forward_offset: float = 0.0,
        derivative_filter_alpha: float = 0.0,
    ):
        """
        A basic PID controller with optional derivative filtering and anti-windup.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            dt: Sampling time step
            min_output: Minimum output clamp (or None for no clamp)
            max_output: Maximum output clamp (or None for no clamp)
            feed_forward_offset: Feed-forward offset (applied to the error signal)
            derivative_filter_alpha: Alpha parameter for first-order low-pass filtering of the derivative term.
                                    0 means no filtering (use the raw derivative),
                                    closer to 1 means heavier filtering (more smoothing).
        """
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.dt: float = dt

        self.min_output: Union[float, None] = min_output
        self.max_output: Union[float, None] = max_output

        self.integral_error: float = 0.0
        self.previous_error: float = 0.0

        self.derivative_filter_alpha: float = derivative_filter_alpha
        self.derivative_filtered: float = 0.0

        self.feed_forward_offset: float = feed_forward_offset

    def update(self, error: float, return_all_terms: bool = False) -> Union[float, tuple[float, float, float, float]]:
        """
        Compute the PID output for the given error signal.

        Args:
            error: Current error (setpoint - measured_value)
            return_all_terms: Whether to return the individual P, I, D terms in addition to the total output

        Returns:
            PID output or (output, p_term, i_term, d_term) if return_all_terms is True
        """
        p_term = self.kp * error

        self.integral_error += error * self.dt
        i_term = self.ki * self.integral_error

        derivative_raw = (error - self.previous_error) / self.dt

        # Filter derivative, if desired
        # alpha=0 => derivative_filtered = derivative_raw (no filtering)
        # alpha=1 => derivative_filtered changes very slowly (heavy filtering)
        self.derivative_filtered = (
            self.derivative_filter_alpha * self.derivative_filtered
            + (1 - self.derivative_filter_alpha) * derivative_raw
        )
        d_term = self.kd * self.derivative_filtered

        output = p_term + i_term + d_term
        output = output + self.feed_forward_offset * np.sign(output)

        self.previous_error = error

        # Apply output clamping, if limits are set
        if self.min_output is not None and output < self.min_output:
            output = self.min_output
            self.integral_error -= error * self.dt  # remove the last integration step to prevent windup

        elif self.max_output is not None and output > self.max_output:
            output = self.max_output
            self.integral_error -= error * self.dt  # remove the last integration step to prevent windup

        if return_all_terms:
            return output, p_term, i_term, d_term

        return output

    def reset(self):
        """
        Reset the PID controller state.
        """
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.derivative_filtered = 0.0

    def set_output_limits(self, min_output: Union[float, None], max_output: Union[float, None]):
        """
        Dynamically set output limits.
        """
        self.min_output = min_output
        self.max_output = max_output
