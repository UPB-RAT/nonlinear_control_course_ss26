class PID:
    """Simple discrete PID controller with anti-windup."""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = None,
                 integral_limit: float = None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit  # ← clamp integral separately

        self._integral   = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        self._integral += error * dt

        # Anti-windup — clamp integral term
        if self.integral_limit is not None:
            self._integral = max(-self.integral_limit,
                                  min(self.integral_limit, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)

        if self.output_limit is not None:
            output = max(-self.output_limit, min(self.output_limit, output))

        return output