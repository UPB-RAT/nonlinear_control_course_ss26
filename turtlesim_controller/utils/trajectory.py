import math

class Figure8Trajectory:

    def __init__(self, a=3.0, center=(5.5, 5.5), speed=0.5):
        self.a = a
        self.cx, self.cy = center
        self.speed = speed
        self.t = 0.0

    def step(self, dt):
        self.t += self.speed * dt

        x = self.a * math.sin(self.t) + self.cx
        y = self.a * math.sin(self.t) * math.cos(self.t) + self.cy

        dx = self.a * math.cos(self.t) * self.speed
        dy = self.a * (math.cos(2*self.t) - math.sin(self.t)**2) * self.speed

        return x, y, dx, dy
    

class SquareTrajectory:

    def __init__(self, size=3.0, center=(5.5, 5.5), speed=0.5, sharpness=1):
        """
        size: half-length of square
        sharpness: higher = sharper corners (but still smooth)
        """
        self.size = size
        self.cx, self.cy = center
        self.speed = speed
        self.t = 0.0
        self.n = sharpness  # controls "square-ness"

    def step(self, dt):
        self.t += self.speed * dt

        # parameterized square using superellipse (Lamé curve)
        # x = cos(t)^(2/n), y = sin(t)^(2/n)

        c = math.cos(self.t)
        s = math.sin(self.t)

        # preserve sign (important!)
        x = self.size * math.copysign(abs(c) ** (2.0 / self.n), c)
        y = self.size * math.copysign(abs(s) ** (2.0 / self.n), s)

        x += self.cx
        y += self.cy

        # derivatives (approx, stable enough for PID feedforward)
        eps = 1e-5
        c2 = math.cos(self.t + eps)
        s2 = math.sin(self.t + eps)

        x2 = self.size * math.copysign(abs(c2) ** (2.0 / self.n), c2)
        y2 = self.size * math.copysign(abs(s2) ** (2.0 / self.n), s2)

        dx = (x2 - x) / eps * self.speed
        dy = (y2 - y) / eps * self.speed

        return x, y, dx, dy