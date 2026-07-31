"""Reusable velocity smoothing for manual Web base commands."""


class VelocityProfile:
    """One-dimensional jerk- and acceleration-limited velocity profile."""

    def __init__(self, max_acceleration: float, max_jerk: float) -> None:
        self._max_acceleration = max_acceleration
        self._max_jerk = max_jerk
        self.velocity = 0.0
        self.acceleration = 0.0

    def update(self, target: float, period: float) -> float:
        """Advance the profile by one bounded time step."""
        period = max(1.0e-4, min(period, 0.1))
        velocity_error = target - self.velocity
        desired_acceleration = max(
            -self._max_acceleration,
            min(self._max_acceleration, velocity_error / period),
        )
        acceleration_step = self._max_jerk * period
        self.acceleration += max(
            -acceleration_step,
            min(
                acceleration_step,
                desired_acceleration - self.acceleration,
            ),
        )
        next_velocity = self.velocity + self.acceleration * period

        reached_target = (
            velocity_error == 0.0
            or (velocity_error > 0.0 and next_velocity >= target)
            or (velocity_error < 0.0 and next_velocity <= target)
        )
        if reached_target:
            self.velocity = target
            self.acceleration = 0.0
        else:
            self.velocity = next_velocity
        return self.velocity

    def reset(self) -> None:
        """Reset velocity and acceleration to a stopped state."""
        self.velocity = 0.0
        self.acceleration = 0.0
