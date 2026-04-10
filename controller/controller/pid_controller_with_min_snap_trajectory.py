import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, Point
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from controller.pid import PID
from controller.waypoint_manager import WaypointManager
from controller.minimum_snap import MinimumSnapTrajectory


class DroneControllerTraj(Node):

    def __init__(self):
        super().__init__('drone_controller_traj')
        self._declare_parameters()

        robot_name        = self.get_parameter('robot_name').value
        self._rate        = self.get_parameter('control_rate').value
        self._takeoff_z   = self.get_parameter('takeoff_height').value
        self._armed       = False

        # ── PID controllers ───────────────────────────────────────────────────
        self._pid = {
            'x':   self._make_pid('pid_x',   'max_linear_vel'),
            'y':   self._make_pid('pid_y',   'max_linear_vel'),
            'z':   self._make_pid('pid_z',   'max_vertical_vel'),
            'yaw': self._make_pid('pid_yaw', 'max_yaw_rate'),
        }

        # ── Trajectory ────────────────────────────────────────────────────────
        wps       = self.get_parameter('waypoints').value
        waypoints = [list(wps[i:i+4]) for i in range(0, len(wps), 4)]

        # Confirm waypoints loaded correctly from launch file
        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints from parameter:')
        for i, wp in enumerate(waypoints):
            self.get_logger().info(f'  wp{i}: {wp}')

        times        = self._allocate_times(waypoints)
        self._traj   = MinimumSnapTrajectory(waypoints, times)
        self._traj_t = None

        self.get_logger().info(
            f'Minimum snap trajectory loaded — '
            f'{len(waypoints)} waypoints, '
            f'total time: {self._traj.total_time:.1f}s')

        # ── State ─────────────────────────────────────────────────────────────
        self._pose   = None
        self._last_t = None
        self._phase  = 'TAKEOFF'   # 'TAKEOFF' → 'TRAJ'

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Twist, f'/{robot_name}/cmd_vel', 10)
        self._arm_pub = self.create_publisher(
            Bool,  f'/{robot_name}/enable', 10)

        self.create_subscription(
            Pose, f'/{robot_name}/pose',
            self._pose_cb, 10)

        # Multi-waypoint trajectory via CSV string
        # "x1,y1,z1,yaw1, x2,y2,z2,yaw2, ..."
        self.create_subscription(
            String, f'/{robot_name}/waypoints',
            self._waypoints_cb, 10)

        # Single goal — full pose
        self.create_subscription(
            Pose, f'/{robot_name}/goal_pose',
            self._goal_pose_cb, 10)

        # Single goal — point only (yaw=0)
        self.create_subscription(
            Point, f'/{robot_name}/goal',
            self._goal_point_cb, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._control_loop)
        self.create_timer(2.0, self._arm_once)

        self.get_logger().info(
            f'DroneControllerTraj ready — arming in 2s\n'
            f'  Takeoff height: {self._takeoff_z:.1f}m\n'
            f'  Send waypoints:\n'
            f'    ros2 topic pub --once /{robot_name}/waypoints '
            f'std_msgs/msg/String '
            f'"{{data: \'0,0,2,0, 2,0,2,0, 2,2,2,0\'}}"\n'
            f'  Send single goal:\n'
            f'    ros2 topic pub --once /{robot_name}/goal '
            f'geometry_msgs/msg/Point "{{x: 2.0, y: 0.0, z: 2.0}}"'
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

    # ── Waypoint / goal callbacks ─────────────────────────────────────────────
    def _waypoints_cb(self, msg: String):
        """
        Receive new waypoint list as flat CSV string at runtime.
        Example:
          ros2 topic pub --once /bebop1/waypoints std_msgs/msg/String \
            "{data: '0,0,2,0, 2,0,2,0, 2,2,2,0, 0,0,2,0'}"
        """
        try:
            vals      = [float(v.strip()) for v in msg.data.split(',')]
            waypoints = [list(vals[i:i+4]) for i in range(0, len(vals), 4)]
            if len(waypoints) < 2:
                self.get_logger().warn('Need at least 2 waypoints')
                return
            self._reset_trajectory(waypoints)
        except Exception as e:
            self.get_logger().error(f'Invalid waypoints string: {e}')

    def _goal_pose_cb(self, msg: Pose):
        """Build 2-point trajectory from current position to goal pose."""
        if self._pose is None:
            return
        yaw     = self._quat_to_yaw(msg.orientation)
        current = [
            self._pose.position.x,
            self._pose.position.y,
            self._pose.position.z,
            self._quat_to_yaw(self._pose.orientation)
        ]
        goal = [msg.position.x, msg.position.y, msg.position.z, yaw]
        self._reset_trajectory([current, goal])
        self.get_logger().info(
            f'New goal_pose: ({goal[0]:.2f}, {goal[1]:.2f}, '
            f'{goal[2]:.2f}, yaw={math.degrees(yaw):.1f}°)')

    def _goal_point_cb(self, msg: Point):
        """Build 2-point trajectory from current position to goal point."""
        if self._pose is None:
            return
        current = [
            self._pose.position.x,
            self._pose.position.y,
            self._pose.position.z,
            self._quat_to_yaw(self._pose.orientation)
        ]
        goal = [msg.x, msg.y, msg.z, 0.0]
        self._reset_trajectory([current, goal])
        self.get_logger().info(
            f'New goal point: ({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})')

    # ── Control loop ──────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self._armed or self._pose is None:
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

        # ── Phase 1: takeoff — climb to takeoff_height before trajectory ──────
        if self._phase == 'TAKEOFF':
            ez = self._takeoff_z - z

            self._fly_to(x, y, z, yaw,
                         x, y, self._takeoff_z, 0.0,
                         dt, 'TAKEOFF')

            if abs(ez) < 0.15:
                self._phase  = 'TRAJ'
                self._traj_t = now.nanoseconds * 1e-9
                [pid.reset() for pid in self._pid.values()]
                self.get_logger().info(
                    f'Takeoff complete at z={z:.2f}m — starting trajectory ✅')
            return

        # ── Phase 2: follow minimum snap trajectory ───────────────────────────
        elapsed = now.nanoseconds * 1e-9 - self._traj_t

        if elapsed >= self._traj.total_time:
            tx, ty, tz, tyaw = self._traj.get_goal(self._traj.total_time)
            label = 'HOVER'
        else:
            tx, ty, tz, tyaw = self._traj.get_goal(elapsed)
            label = f'TRAJ t={elapsed:.1f}/{self._traj.total_time:.1f}s'

        self._fly_to(x, y, z, yaw, tx, ty, tz, tyaw, dt, label)

    # ── Fly to target ─────────────────────────────────────────────────────────
    def _fly_to(self, x, y, z, yaw, tx, ty, tz, tyaw, dt, label=''):
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
            f'[{label}] '
            f'target=({tx:.2f},{ty:.2f},{tz:.2f}) '
            f'pos=({x:.2f},{y:.2f},{z:.2f}) '
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
    def _reset_trajectory(self, waypoints: list):
        times        = self._allocate_times(waypoints)
        self._traj   = MinimumSnapTrajectory(waypoints, times)
        self._traj_t = self.get_clock().now().nanoseconds * 1e-9
        self._phase  = 'TRAJ'   # skip takeoff on runtime updates
        [pid.reset() for pid in self._pid.values()]
        self.get_logger().info(
            f'Trajectory reset: {len(waypoints)} waypoints, '
            f'{self._traj.total_time:.1f}s')

    @staticmethod
    def _allocate_times(waypoints: list, avg_speed: float = 0.5) -> list:
        times = []
        for i in range(len(waypoints) - 1):
            dx   = waypoints[i+1][0] - waypoints[i][0]
            dy   = waypoints[i+1][1] - waypoints[i][1]
            dz   = waypoints[i+1][2] - waypoints[i][2]
            dist = max(math.sqrt(dx**2 + dy**2 + dz**2), 0.1)
            times.append(max(dist / avg_speed, 2.0))
        return times

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
        self.declare_parameter('max_linear_vel',   0.5)
        self.declare_parameter('max_vertical_vel', 0.08)
        self.declare_parameter('max_yaw_rate',     1.0)
        self.declare_parameter('takeoff_height',   2.0)

        # DOUBLE_ARRAY so ROS2 accepts any length list from launch file
        self.declare_parameter(
            'waypoints',
            [0.000,  0.000,  2.000,  0.0000,
             0.000,  0.000,  2.000,  0.0000],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='Flat list [x,y,z,yaw, x,y,z,yaw, ...]'
            )
        )

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
    node = DroneControllerTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()