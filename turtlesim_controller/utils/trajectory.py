import math
import numpy as np
from scipy.interpolate import CubicSpline
import json


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

        # parameterized square using superellipse
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
    
# class WaypointTrajectory:
#     def __init__(self, json_file, speed=1.0):

#         self.speed = speed
#         self.t = 0.0

#         # ---------------------------
#         # Load waypoints
#         # ---------------------------
#         with open(json_file, 'r') as f:
#             data = json.load(f)

#         pts = np.array(data["waypoints"], dtype=float)

#         if len(pts) < 2:
#             raise ValueError("Trajectory needs at least 2 waypoints")

#         x = pts[:, 0]
#         y = pts[:, 1]

#         # ---------------------------
#         # Arc-length parameterization
#         # ---------------------------
#         diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
#         s = np.insert(np.cumsum(diffs), 0, 0.0)

#         self.s_max = s[-1]

#         # avoid divide-by-zero if all points identical
#         if self.s_max == 0:
#             raise ValueError("Waypoints must not all be identical")

#         # ---------------------------
#         # Smooth spline path
#         # ---------------------------
#         self.xs = CubicSpline(s, x)
#         self.ys = CubicSpline(s, y)

#     def step(self, dt):

#         # advance along arc-length
#         self.t += self.speed * dt
#         self.t = min(self.t, self.s_max)

#         # ---------------------------
#         # Position
#         # ---------------------------
#         x = self.xs(self.t)
#         y = self.ys(self.t)

#         # ---------------------------
#         # Velocity (tangent)
#         # ---------------------------
#         dx = self.xs(self.t, 1) * self.speed
#         dy = self.ys(self.t, 1) * self.speed

#         return float(x), float(y), float(dx), float(dy)

class WaypointTrajectory:
    def __init__(self, json_file, speed=1.0):

        self.speed = speed
        self.t = 0.0

        # ---------------------------
        # Load waypoints
        # ---------------------------
        with open(json_file, 'r') as f:
            data = json.load(f)

        pts = np.array(data["waypoints"], dtype=float)

        if len(pts) < 3:
            raise ValueError("Need at least 3 waypoints for periodic spline")

        # ---------------------------
        # FORCE CLOSED LOOP (important for periodic spline)
        # ---------------------------
        if not np.allclose(pts[0], pts[-1]):
            pts = np.vstack([pts, pts[0]])

        x = pts[:, 0]
        y = pts[:, 1]

        # ---------------------------
        # Arc-length parameterization
        # ---------------------------
        diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.insert(np.cumsum(diffs), 0, 0.0)

        self.s_max = s[-1]

        if self.s_max == 0:
            raise ValueError("Invalid waypoints (all identical)")

        # ---------------------------
        # PERIODIC SPLINES
        # ---------------------------
        self.xs = CubicSpline(s, x, bc_type="periodic")
        self.ys = CubicSpline(s, y, bc_type="periodic")

    def step(self, dt):

        # ---------------------------
        # wrap-around motion (infinite loop)
        # ---------------------------
        self.t += self.speed * dt
        self.t = self.t % self.s_max

        # ---------------------------
        # Position
        # ---------------------------
        x = self.xs(self.t)
        y = self.ys(self.t)

        # ---------------------------
        # Velocity (tangent)
        # ---------------------------
        dx = self.xs(self.t, 1) * self.speed
        dy = self.ys(self.t, 1) * self.speed

        return float(x), float(y), float(dx), float(dy)