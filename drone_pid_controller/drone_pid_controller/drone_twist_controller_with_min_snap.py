import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from rcl_interfaces.msg import SetParametersResult

from drone_pid_controller.pid import PID
from drone_pid_controller.min_snap_trajectory import MinimumSnapTrajectory

MAX_VEL = 3.0


class QuadcopterPIDMinSnap(Node):

    def __init__(self):
        super().__init__('quadcopter_pid_with_snap')

        self.path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.pub = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)
        self.sub = self.create_subscription(
            PoseArray,
            '/world/quadcopter/pose/info',
            self.pose_callback,
            10
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        self.pid_x = PID(1.2, 0.0, 0.4)
        self.pid_y = PID(1.2, 0.0, 0.4)
        self.pid_z = PID(1.8, 0.0, 0.5)

        self.declare_parameter("trajectory", "figure8")
        self.declare_parameter("A", 2.0)
        self.declare_parameter("w", 0.3)
        self.declare_parameter("height", 1.0)
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("climb_rate", 0.1)
        self.declare_parameter("side", 4.0)

        self.traj_type = self.get_parameter("trajectory").value

        self.current_pose = None

        self.last_time = None
        self.start_time = None

        self.traj = self._build_trajectory()

        self.add_on_set_parameters_callback(self.param_callback)

        self.create_timer(0.05, self.control_loop)

    def _build_trajectory(self):
        params = {
            "A":     self.get_parameter("A").value,
            "r":     self.get_parameter("radius").value,
            "h":     self.get_parameter("height").value,
            "climb": self.get_parameter("climb_rate").value,
            "side":  self.get_parameter("side").value,
        }
        return MinimumSnapTrajectory(
            shape=self.traj_type,
            shape_params=params,
            times=2.0,
            num_points=30
        )

    def pose_callback(self, msg):
        self.current_pose = msg.poses[1]

        if self.last_time is None:
            now = self.get_clock().now()
            self.last_time = now
            self.start_time = now

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose = self.current_pose

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def param_callback(self, params):
        for p in params:
            if p.name == "trajectory":
                self.traj_type = p.value
                self.traj = self._build_trajectory()

                self.start_time = self.get_clock().now()

                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()

                self.get_logger().info(
                    f"Switched minimum snap trajectory → {self.traj_type}"
                )

        return SetParametersResult(successful=True)

    def control_loop(self):
        if self.current_pose is None or self.last_time is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            return

        self.last_time = now

        t = (now - self.start_time).nanoseconds / 1e9

        tx, ty, tz, _ = self.traj.get_goal(t)
        der = self.traj.get_derivatives(t)
        vx_ff, vy_ff, vz_ff = der["vel"]

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z

        vx = vx_ff + self.pid_x.compute(tx - x, dt)
        vy = vy_ff + self.pid_y.compute(ty - y, dt)
        vz = vz_ff + self.pid_z.compute(tz - z, dt)

        vx = max(-MAX_VEL, min(MAX_VEL, vx))
        vy = max(-MAX_VEL, min(MAX_VEL, vy))
        vz = max(-MAX_VEL, min(MAX_VEL, vz))

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        self.pub.publish(cmd)

        self.get_logger().info(
            f"[{self.traj_type}] "
            f"T=({tx:.2f}, {ty:.2f}, {tz:.2f}) "
            f"P=({x:.2f}, {y:.2f}, {z:.2f})",
            throttle_duration_sec=0.5
        )


def main():
    rclpy.init()
    node = QuadcopterPIDMinSnap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()