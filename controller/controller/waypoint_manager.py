import math


class WaypointManager:
    """Manages a list of [x, y, z, yaw] waypoints."""

    def __init__(self, waypoints: list, xy_tol: float,
                 z_tol: float, yaw_tol: float):
        self.waypoints  = waypoints
        self.xy_tol     = xy_tol
        self.z_tol      = z_tol
        self.yaw_tol    = yaw_tol
        self._index     = 0
        self.completed  = False

    @property
    def current(self):
        """Returns current target waypoint [x, y, z, yaw]."""
        return self.waypoints[self._index]

    @property
    def last(self):
        """Returns the last waypoint — used for hovering after completion."""
        return self.waypoints[-1]

    def check_arrival(self, x: float, y: float,
                      z: float, yaw: float) -> bool:
        wx, wy, wz, wyaw = self.current

        xy_err  = math.sqrt((x - wx) ** 2 + (y - wy) ** 2)
        z_err   = abs(z - wz)
        yaw_err = abs(self._wrap_angle(yaw - wyaw))

        return (xy_err  < self.xy_tol and
                z_err   < self.z_tol  and
                yaw_err < self.yaw_tol)

    def advance(self, logger=None):
        if self._index < len(self.waypoints) - 1:
            self._index += 1
            if logger:
                logger.info(
                    f'Waypoint reached → next: {self.current}'
                )
        else:
            self.completed = True
            if logger:
                logger.info('All waypoints completed! Hovering at last position.')

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle