import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist, PoseArray

from drone_pid_controller.pid import PID
from drone_pid_controller.trajectory import get_target
from rcl_interfaces.msg import SetParametersResult


class QuadcopterPID(Node):

    def __init__(self):
        super().__init__('quadcopter_pid')

        self.path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.pub = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)
        self.sub = self.create_subscription(PoseArray, '/world/quadcopter/pose/info', self.pose_callback, 10)

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
        self.declare_parameter("speed", 1.0)

        self.traj_type = self.get_parameter("trajectory").value

        self.current_pose = None
        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.param_callback)

        self.create_timer(0.05, self.control_loop)

    def pose_callback(self, msg):

        self.current_pose = msg.poses[1]

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
                self.get_logger().info(
                    f"Switched trajectory → {self.traj_type}"
                )

        return SetParametersResult(successful=True)

    def control_loop(self):

        if self.current_pose is None:
            return

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        self.last_time = now

        t = (now - self.start_time).nanoseconds / 1e9

        params = {
            "A": self.get_parameter("A").value,
            "w": self.get_parameter("w").value,
            "height": self.get_parameter("height").value,

            "radius": self.get_parameter("radius").value,
            "climb_rate": self.get_parameter("climb_rate").value,

            "side": self.get_parameter("side").value,
            "speed": self.get_parameter("speed").value
        }

        tx, ty, tz = get_target(self.traj_type, params, t)

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z

        err_x = tx - x
        err_y = ty - y
        err_z = tz - z

        vx = self.pid_x.compute(err_x, dt)
        vy = self.pid_y.compute(err_y, dt)
        vz = self.pid_z.compute(err_z, dt)

        cmd = Twist()

        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz

        self.pub.publish(cmd)

        self.get_logger().info(
            f"[{self.traj_type}] "
            f"Target=({tx:.2f}, {ty:.2f}, {tz:.2f}) "
            f"Pos=({x:.2f}, {y:.2f}, {z:.2f})",
            throttle_duration_sec=0.5
        )


def main():
    rclpy.init()
    node = QuadcopterPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()