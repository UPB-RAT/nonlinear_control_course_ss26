class PID:
    def __init__(self, kp, ki, kd, dt, limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.limit = limit

        self.integral = 0.0
        self.prev_error = 0.0

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.derivative = 0.0
        self.raw_output = 0.0
        self.output = 0.0
        self.saturated = False

    def compute(self, error):
        self.integral += error * self.dt
        self.derivative = (error - self.prev_error) / self.dt

        self.p_term = self.kp * error
        self.i_term = self.ki * self.integral
        self.d_term = self.kd * self.derivative

        self.raw_output = self.p_term + self.i_term + self.d_term
        self.output = self.raw_output

        self.prev_error = error

        if self.limit is not None:
            limited = max(-self.limit, min(self.limit, self.output))
            self.saturated = limited != self.output
            self.output = limited

        return self.output

    def set_gains(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd

    def reset_memory(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def snapshot(self):
        return {
            "p": self.p_term,
            "i": self.i_term,
            "d": self.d_term,
            "derivative": self.derivative,
            "raw_output": self.raw_output,
            "output": self.output,
            "integral": self.integral,
            "saturated": self.saturated,
        }