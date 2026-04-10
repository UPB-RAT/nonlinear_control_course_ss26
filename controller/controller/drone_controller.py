import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
from std_msgs.msg import Bool

from controller.pid import PID
from controller.waypoint_manager import WaypointManager


class DroneController(Node):

    def __init__(self):
        super().__init__('drone_controller')
        self._declare_parameters()

        robot_name   = self.get_parameter('robot_name').value
        self._rate   = self.get_parameter('control_rate').value
        self._armed  = False
        self._goal   = None  # [x, y, z, yaw] — set via topic or default waypoint

        # ── PID controllers ───────────────────────────────────────────────────
        self._pid = {
            'x':   self._make_pid('pid_x',   'max_linear_vel'),
            'y':   self._make_pid('pid_y',   'max_linear_vel'),
            'z':   self._make_pid('pid_z',   'max_vertical_vel'),
            'yaw': self._make_pid('pid_yaw', 'max_yaw_rate'),
        }

        # ── Waypoint manager (optional multi-waypoint mode) ───────────────────
        wps = self.get_parameter('waypoints').value
        waypoints = [list(wps[i:i+4]) for i in range(0, len(wps), 4)]
        self._wpm = WaypointManager(
            waypoints,
            xy_tol  = self.get_parameter('xy_tolerance').value,
            z_tol   = self.get_parameter('z_tolerance').value,
            yaw_tol = self.get_parameter('yaw_tolerance').value,
        )

        # Set initial goal from first waypoint
        self._goal = list(waypoints[0]) if waypoints else None

        # ── State ─────────────────────────────────────────────────────────────
        self._pose   = None
        self._last_t = None

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Twist, f'/{robot_name}/cmd_vel', 10)
        self._arm_pub = self.create_publisher(
            Bool, f'/{robot_name}/enable', 10)

        self.create_subscription(
            Pose, f'/{robot_name}/pose',
            self._pose_cb, 10)

        # Goal from terminal or GUI — accepts full Pose (position + yaw)
        self.create_subscription(
            Pose, f'/{robot_name}/goal_pose',
            self._goal_pose_cb, 10)

        # Convenience: accept Point (x, y, z) with yaw=0
        self.create_subscription(
            Point, f'/{robot_name}/goal',
            self._goal_point_cb, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._control_loop)
        self.create_timer(2.0, self._arm_once)

        self.get_logger().info(
            f'DroneController ready — arming in 2s\n'
            f'  Initial goal: {self._goal}\n'
            f'  Send new goals via:\n'
            f'    ros2 topic pub --once /{robot_name}/goal_pose '
            f'geometry_msgs/msg/Pose '
            f'"{{position: {{x: 1.0, y: 0.0, z: 1.5}}, orientation: {{w: 1.0}}}}"\n'
            f'    ros2 topic pub --once /{robot_name}/goal '
            f'geometry_msgs/msg/Point "{{x: 1.0, y: 0.0, z: 1.5}}"'
        )

    # ── Arming ────────────────────────────────────────────────────────────────
    def _arm_once(self):
        if not self._armed:
            msg = Bool()
            msg.data = True
            self._arm_pub.publish(msg)
            self._armed = True
            self.get_logger().info('Drone armed ✅')

    # ── Pose callback ─────────────────────────────────────────────────────────
    def _pose_cb(self, msg: Pose):
        self._pose = msg

    # ── Goal callbacks ────────────────────────────────────────────────────────
    def _goal_pose_cb(self, msg: Pose):
        """Receive full goal pose (position + orientation) from GUI or terminal."""
        yaw = self._quat_to_yaw(msg.orientation)
        self._goal = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
            yaw
        ]
        [pid.reset() for pid in self._pid.values()]
        self.get_logger().info(
            f'New goal_pose received: '
            f'({self._goal[0]:.2f}, {self._goal[1]:.2f}, '
            f'{self._goal[2]:.2f}, yaw={math.degrees(yaw):.1f}°)')

    def _goal_point_cb(self, msg: Point):
        """Receive simple x/y/z goal (yaw unchanged) from terminal."""
        yaw = self._goal[3] if self._goal else 0.0
        self._goal = [msg.x, msg.y, msg.z, yaw]
        [pid.reset() for pid in self._pid.values()]
        self.get_logger().info(
            f'New goal received: '
            f'({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

    # ── Control loop ──────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self._armed or self._pose is None or self._goal is None:
            return

        now = self.get_clock().now()
        if self._last_t is None:
            self._last_t = now
            return

        dt = (now - self._last_t).nanoseconds * 1e-9
        dt = min(max(dt, 1e-4), 0.05)
        self._last_t = now

        x   = self._pose.position.x
        y   = self._pose.position.y
        z   = self._pose.position.z
        yaw = self._quat_to_yaw(self._pose.orientation)

        tx, ty, tz, tyaw = self._goal
        self._fly_to(x, y, z, yaw, tx, ty, tz, tyaw, dt)

    # ── Fly to target ─────────────────────────────────────────────────────────
    def _fly_to(self, x, y, z, yaw, tx, ty, tz, tyaw, dt):
        # World → body frame errors
        ex_world = tx - x
        ey_world = ty - y
        ex   =  ex_world * math.cos(yaw) + ey_world * math.sin(yaw)
        ey   = -ex_world * math.sin(yaw) + ey_world * math.cos(yaw)
        ez   = tz - z
        eyaw = WaypointManager._wrap_angle(tyaw - yaw)

        vx = self._pid['x'].compute(ex,    dt)
        vy = self._pid['y'].compute(ey,    dt)
        vz = self._pid['z'].compute(ez,    dt)
        wz = self._pid['yaw'].compute(eyaw, dt)

        dist = math.sqrt(ex_world**2 + ey_world**2 + ez**2)

        self.get_logger().info(
            f'[FLY] target=({tx:.2f},{ty:.2f},{tz:.2f}) '
            f'pos=({x:.2f},{y:.2f},{z:.2f}) '
            f'err=({ex_world:.2f},{ey_world:.2f},{ez:.2f}) '
            f'dist={dist:.2f}m '
            f'cmd=({vx:.2f},{vy:.2f},{vz:.2f})',
            throttle_duration_sec=0.5
        )

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = vy
        cmd.linear.z  = vz
        cmd.angular.z = wz
        self._cmd_pub.publish(cmd)

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _make_pid(self, prefix: str, limit_param: str) -> PID:
        kp    = self.get_parameter(f'{prefix}.kp').value
        ki    = self.get_parameter(f'{prefix}.ki').value
        kd    = self.get_parameter(f'{prefix}.kd').value
        limit = self.get_parameter(limit_param).value
        return PID(kp=kp, ki=ki, kd=kd,
                   output_limit=limit,
                   integral_limit=limit * 0.5)

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _declare_parameters(self):
        self.declare_parameter('robot_name',       'bebop1')
        self.declare_parameter('control_rate',     50.0)
        self.declare_parameter('xy_tolerance',     0.15)
        self.declare_parameter('z_tolerance',      0.10)
        self.declare_parameter('yaw_tolerance',    0.10)
        self.declare_parameter('max_linear_vel',   0.5)
        self.declare_parameter('max_vertical_vel', 0.08)
        self.declare_parameter('max_yaw_rate',     1.0)
        self.declare_parameter('waypoints', [
            0.0, 0.0, 1.0, 0.0,   # default hover point
        ])
        self.declare_parameter('pid_x.kp',   0.5)
        self.declare_parameter('pid_x.ki',   0.08)
        self.declare_parameter('pid_x.kd',   0.4)
        self.declare_parameter('pid_y.kp',   0.5)
        self.declare_parameter('pid_y.ki',   0.08)
        self.declare_parameter('pid_y.kd',   0.4)
        self.declare_parameter('pid_z.kp',   0.2)
        self.declare_parameter('pid_z.ki',   0.02)
        self.declare_parameter('pid_z.kd',   0.1)
        self.declare_parameter('pid_yaw.kp', 1.0)
        self.declare_parameter('pid_yaw.ki', 0.01)
        self.declare_parameter('pid_yaw.kd', 0.1)


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()