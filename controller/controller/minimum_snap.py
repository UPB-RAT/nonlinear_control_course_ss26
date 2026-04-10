import numpy as np


class MinimumSnapTrajectory:
    """
    Generates minimum snap piecewise polynomial trajectory
    through a list of 3D waypoints.

    Reference: Mellinger & Kumar, ICRA 2011
    """

    POLY_DEG  = 7   # degree 7 → 8 coefficients per segment
    DERIV_MIN = 4   # minimize 4th derivative (snap)

    def __init__(self, waypoints: list, times: list):
        """
        waypoints : list of [x, y, z, yaw]  — N+1 points
        times     : list of segment durations [T1, T2, ..., TN] in seconds
        """
        assert len(times) == len(waypoints) - 1, \
            "Need N-1 segment times for N waypoints"

        self._waypoints = np.array(waypoints)   # (N+1, 4)
        self._times     = np.array(times)        # (N,)
        self._t_start   = np.concatenate([[0], np.cumsum(times)])  # absolute times

        # Solve for x, y, z independently (yaw is linearly interpolated)
        self._coeffs = {
            'x': self._solve(self._waypoints[:, 0]),
            'y': self._solve(self._waypoints[:, 1]),
            'z': self._solve(self._waypoints[:, 2]),
        }
        self._total_time = self._t_start[-1]

    # ── Public API ────────────────────────────────────────────────────────────

    def get_goal(self, t: float) -> list:
        """
        Returns [x, y, z, yaw] at time t seconds from trajectory start.
        Clamps to endpoints if t is out of range.
        """
        t = max(0.0, min(t, self._total_time))
        seg, tau = self._get_segment(t)

        x   = self._eval(self._coeffs['x'][seg], tau, deriv=0)
        y   = self._eval(self._coeffs['y'][seg], tau, deriv=0)
        z   = self._eval(self._coeffs['z'][seg], tau, deriv=0)
        yaw = self._interp_yaw(t)

        return [x, y, z, yaw]

    def get_derivatives(self, t: float) -> dict:
        """
        Returns position, velocity, acceleration at time t.
        Useful for feedforward control.
        """
        t = max(0.0, min(t, self._total_time))
        seg, tau = self._get_segment(t)

        return {
            'pos': [
                self._eval(self._coeffs['x'][seg], tau, 0),
                self._eval(self._coeffs['y'][seg], tau, 0),
                self._eval(self._coeffs['z'][seg], tau, 0),
            ],
            'vel': [
                self._eval(self._coeffs['x'][seg], tau, 1),
                self._eval(self._coeffs['y'][seg], tau, 1),
                self._eval(self._coeffs['z'][seg], tau, 1),
            ],
            'acc': [
                self._eval(self._coeffs['x'][seg], tau, 2),
                self._eval(self._coeffs['y'][seg], tau, 2),
                self._eval(self._coeffs['z'][seg], tau, 2),
            ],
        }

    @property
    def total_time(self):
        return self._total_time

    @property
    def completed(self, t: float) -> bool:
        return t >= self._total_time

    # ── Solver ────────────────────────────────────────────────────────────────

    def _solve(self, positions: np.ndarray) -> np.ndarray:
        """
        Solve for polynomial coefficients for one axis.
        Returns array of shape (N_segments, 8).
        """
        N   = len(self._times)      # number of segments
        n   = self.POLY_DEG + 1     # coefficients per segment = 8
        dim = N * n                 # total unknowns

        Q = np.zeros((dim, dim))
        A = np.zeros((0, dim))
        b = np.zeros(0)

        # ── Cost matrix Q (minimize snap = 4th derivative) ───────────────────
        for seg in range(N):
            T   = self._times[seg]
            Q_s = self._cost_matrix(T)
            i0  = seg * n
            Q[i0:i0+n, i0:i0+n] += Q_s

        # ── Constraints ───────────────────────────────────────────────────────
        # Start: pos, vel, acc, jerk = 0 (or first waypoint pos)
        A, b = self._add_constraint(A, b, seg=0, tau=0.0, deriv=0,
                                    val=positions[0], n=n, N=N)
        A, b = self._add_constraint(A, b, seg=0, tau=0.0, deriv=1, val=0.0, n=n, N=N)
        A, b = self._add_constraint(A, b, seg=0, tau=0.0, deriv=2, val=0.0, n=n, N=N)
        A, b = self._add_constraint(A, b, seg=0, tau=0.0, deriv=3, val=0.0, n=n, N=N)

        # End: pos, vel, acc, jerk = 0 (or last waypoint pos)
        T_last = self._times[-1]
        A, b = self._add_constraint(A, b, seg=N-1, tau=T_last, deriv=0,
                                    val=positions[-1], n=n, N=N)
        A, b = self._add_constraint(A, b, seg=N-1, tau=T_last, deriv=1, val=0.0, n=n, N=N)
        A, b = self._add_constraint(A, b, seg=N-1, tau=T_last, deriv=2, val=0.0, n=n, N=N)
        A, b = self._add_constraint(A, b, seg=N-1, tau=T_last, deriv=3, val=0.0, n=n, N=N)

        # Intermediate waypoints: position + continuity of vel, acc, jerk, snap
        for i in range(1, N):
            T_i = self._times[i-1]
            # Position at end of segment i-1
            A, b = self._add_constraint(A, b, seg=i-1, tau=T_i, deriv=0,
                                        val=positions[i], n=n, N=N)
            # Position at start of segment i
            A, b = self._add_constraint(A, b, seg=i, tau=0.0, deriv=0,
                                        val=positions[i], n=n, N=N)
            # Continuity: vel, acc, jerk, snap at junction
            for deriv in range(1, 5):
                row = np.zeros(dim)
                row[(i-1)*n:(i-1)*n+n] =  self._poly_deriv_coeffs(T_i, deriv, n)
                row[i*n:i*n+n]         = -self._poly_deriv_coeffs(0.0, deriv, n)
                A = np.vstack([A, row])
                b = np.append(b, 0.0)

        # ── Solve via QP: min x'Qx s.t. Ax=b ─────────────────────────────────
        # Using least-squares with constraint via KKT conditions
        n_eq  = A.shape[0]
        K     = np.block([[2*Q, A.T], [A, np.zeros((n_eq, n_eq))]])
        rhs   = np.concatenate([np.zeros(dim), b])

        try:
            sol = np.linalg.solve(K, rhs)
        except np.linalg.LinAlgError:
            sol = np.linalg.lstsq(K, rhs, rcond=None)[0]

        coeffs = sol[:dim].reshape(N, n)
        return coeffs

    def _cost_matrix(self, T: float) -> np.ndarray:
        """
        Integral of (d^4 p/dt^4)^2 dt for degree-7 polynomial over [0, T].
        """
        n = self.POLY_DEG + 1
        Q = np.zeros((n, n))
        r = self.DERIV_MIN

        for i in range(r, n):
            for j in range(r, n):
                # coefficient of integral of p^(r)_i * p^(r)_j
                exp = i + j - 2*r + 1
                coeff_i = self._deriv_factor(i, r)
                coeff_j = self._deriv_factor(j, r)
                Q[i, j] = coeff_i * coeff_j * T**exp / exp
        return Q

    def _add_constraint(self, A, b, seg, tau, deriv, val, n, N):
        row = np.zeros(N * n)
        row[seg*n:seg*n+n] = self._poly_deriv_coeffs(tau, deriv, n)
        A = np.vstack([A, row]) if A.shape[0] > 0 else row.reshape(1, -1)
        b = np.append(b, val)
        return A, b

    @staticmethod
    def _deriv_factor(power: int, deriv: int) -> float:
        """Compute the coefficient after taking `deriv` derivatives of t^power."""
        factor = 1.0
        for i in range(deriv):
            factor *= (power - i)
        return factor

    @staticmethod
    def _poly_deriv_coeffs(tau: float, deriv: int, n: int) -> np.ndarray:
        """Evaluate the deriv-th derivative basis vector at tau."""
        coeffs = np.zeros(n)
        for i in range(deriv, n):
            factor = 1.0
            for k in range(deriv):
                factor *= (i - k)
            coeffs[i] = factor * tau**(i - deriv)
        return coeffs

    @staticmethod
    def _eval(coeffs: np.ndarray, tau: float, deriv: int) -> float:
        """Evaluate polynomial or its derivative at tau."""
        n      = len(coeffs)
        basis  = MinimumSnapTrajectory._poly_deriv_coeffs(tau, deriv, n)
        return float(np.dot(coeffs, basis))

    def _get_segment(self, t: float):
        """Returns (segment_index, local_time tau) for global time t."""
        for i in range(len(self._times)):
            if t <= self._t_start[i+1]:
                tau = t - self._t_start[i]
                return i, tau
        return len(self._times) - 1, self._times[-1]

    def _interp_yaw(self, t: float) -> float:
        """Linearly interpolate yaw between waypoints."""
        seg, tau = self._get_segment(t)
        T    = self._times[seg]
        yaw0 = self._waypoints[seg,   3]
        yaw1 = self._waypoints[seg+1, 3]
        # Wrap-aware interpolation
        diff = yaw1 - yaw0
        while diff >  np.pi: diff -= 2*np.pi
        while diff < -np.pi: diff += 2*np.pi
        return yaw0 + (tau / T) * diff