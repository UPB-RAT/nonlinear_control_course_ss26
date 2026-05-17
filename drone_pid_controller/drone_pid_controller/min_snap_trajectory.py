import math
import numpy as np


class MinimumSnapTrajectory:

    POLY_DEG = 7
    N_COEFFS = POLY_DEG + 1

    def __init__(self, shape: str = "circle", shape_params: dict = None,
                 times: float = 2.0, num_points: int = 10):

        self.shape = shape
        self.params = shape_params or {}
        self.num_points = num_points

        self.waypoints = self._generate_shape(shape, self.params)

        self.times = np.ones(len(self.waypoints) - 1) * times

        self._t_start = np.concatenate([[0], np.cumsum(self.times)])
        self._total_time = self._t_start[-1]

        self._is_loop = np.allclose(self.waypoints[0], self.waypoints[-1], atol=1e-6)

        self.coeffs = {
            "x": self._solve(self.waypoints[:, 0]),
            "y": self._solve(self.waypoints[:, 1]),
            "z": self._solve(self.waypoints[:, 2]),
        }

    def _generate_shape(self, shape, p):
        if shape == "circle":
            return self._circle(p.get("r", 2.0), p.get("h", 1.0), self.num_points)
        elif shape == "figure8":
            return self._figure8(p.get("A", 2.0), p.get("h", 1.0), self.num_points)
        elif shape == "square":
            return self._square(p.get("side", 4.0), p.get("h", 1.0), self.num_points)
        elif shape == "helix":
            return self._helix(p.get("r", 2.0), p.get("climb", 3.0),
                               p.get("h", 1.0), self.num_points)
        else:
            return self._circle(2.0, 1.0, self.num_points)

    def _circle(self, r, h, N):
        w = []
        for i in range(N):
            t = 2 * math.pi * i / N
            w.append([r * math.cos(t), r * math.sin(t), h])
        w.append(w[0])           # close the loop
        return np.array(w)

    def _figure8(self, A, h, N):
        w = []
        for i in range(N):
            t = 2 * math.pi * i / N
            w.append([A * math.sin(t), A * math.sin(t) * math.cos(t), h])
        w.append(w[0])           # close the loop
        return np.array(w)

    def _square(self, side, h, N):
        s = side / 2.0
        corners = [
            np.array([-s, -s, h]),
            np.array([ s, -s, h]),
            np.array([ s,  s, h]),
            np.array([-s,  s, h]),
        ]

        w = []
        for side_idx in range(4):
            p0 = corners[side_idx]
            p1 = corners[(side_idx + 1) % 4]
            pts = N // 4 + (1 if side_idx < N % 4 else 0)
            for j in range(pts):
                a = j / pts
                w.append((1.0 - a) * p0 + a * p1)

        w.append(w[0])           # close the loop
        return np.array(w)

    def _helix(self, r, climb, h, N):
        w = []
        for i in range(N):
            t = 2 * math.pi * i / N
            w.append([r * math.cos(t),
                      r * math.sin(t),
                      h + climb * i / N])
        return np.array(w)

    def get_goal(self, t: float):
        t = self._wrap_or_clamp(t)
        seg, tau = self._get_segment(t)
        x = self._eval(self.coeffs["x"][seg], tau, 0)
        y = self._eval(self.coeffs["y"][seg], tau, 0)
        z = self._eval(self.coeffs["z"][seg], tau, 0)
        return [x, y, z, 0.0]

    def get_derivatives(self, t: float):
        t = self._wrap_or_clamp(t)
        seg, tau = self._get_segment(t)
        return {
            "pos": [self._eval(self.coeffs[ax][seg], tau, 0) for ax in "xyz"],
            "vel": [self._eval(self.coeffs[ax][seg], tau, 1) for ax in "xyz"],
            "acc": [self._eval(self.coeffs[ax][seg], tau, 2) for ax in "xyz"],
        }

    def _wrap_or_clamp(self, t: float) -> float:
        if self._is_loop:
            return t % self._total_time
        return float(np.clip(t, 0.0, self._total_time))

    def _solve(self, positions):
        N = len(self.times)
        n = self.N_COEFFS
        dim = N * n

        Q = np.zeros((dim, dim))
        A = np.zeros((0, dim))
        b = np.zeros(0)

        for i in range(N):
            Q_i = self._cost_matrix(self.times[i])
            Q[i*n:(i+1)*n, i*n:(i+1)*n] += Q_i

        A, b = self._add(A, b, 0, 0, 0, positions[0])
        A, b = self._add(A, b, 0, 0, 1, 0)
        A, b = self._add(A, b, 0, 0, 2, 0)
        A, b = self._add(A, b, 0, 0, 3, 0)

        A, b = self._add(A, b, N-1, self.times[-1], 0, positions[-1])
        A, b = self._add(A, b, N-1, self.times[-1], 1, 0)
        A, b = self._add(A, b, N-1, self.times[-1], 2, 0)
        A, b = self._add(A, b, N-1, self.times[-1], 3, 0)

        for i in range(1, N):
            A, b = self._add(A, b, i-1, self.times[i-1], 0, positions[i])

        for i in range(1, N):
            T = self.times[i-1]
            for d in range(4):
                row = np.zeros(dim)
                row[(i-1)*n:i*n] = self._basis(T, d)
                row[i*n:(i+1)*n] = -self._basis(0, d)
                A = np.vstack([A, row])
                b = np.append(b, 0)

        K = np.block([
            [2 * Q,                        A.T],
            [A,     np.zeros((len(b), len(b)))]
        ])
        rhs = np.concatenate([np.zeros(dim), b])
        sol = np.linalg.lstsq(K, rhs, rcond=None)[0]
        return sol[:dim].reshape(N, n)

    def _cost_matrix(self, T):
        n = self.N_COEFFS
        Q = np.zeros((n, n))
        for i in range(4, n):
            for j in range(4, n):
                ci = self._factor(i, 4)
                cj = self._factor(j, 4)
                power = i + j - 7
                Q[i, j] = ci * cj * T**(power + 1) / (power + 1)
        return Q

    def _factor(self, p, d):
        f = 1
        for i in range(d):
            f *= (p - i)
        return f

    def _basis(self, t, d):
        b = np.zeros(self.N_COEFFS)
        for i in range(d, self.N_COEFFS):
            f = 1
            for k in range(d):
                f *= (i - k)
            b[i] = f * t**(i - d)
        return b

    def _eval(self, coeffs, t, d):
        return float(np.dot(coeffs, self._basis(t, d)))

    def _add(self, A, b, seg, t, d, val):
        row = np.zeros(self.N_COEFFS * len(self.times))
        row[seg*self.N_COEFFS:(seg+1)*self.N_COEFFS] = self._basis(t, d)
        A = np.vstack([A, row]) if len(A) else row.reshape(1, -1)
        b = np.append(b, val)
        return A, b

    def _get_segment(self, t):
        for i in range(len(self.times)):
            if t <= self._t_start[i + 1]:
                return i, t - self._t_start[i]
        return len(self.times) - 1, self.times[-1]