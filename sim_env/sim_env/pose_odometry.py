import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose


class PoseOdom(Node):
    def __init__(self):
        super().__init__('pose_odometry')
        self.declare_parameter('robot_name', 'bebop1')
        self.declare_parameter('log_rate', 10.0)  # Hz — configurable from launch

        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        log_rate   = self.get_parameter('log_rate').get_parameter_value().double_value

        self._log_interval = 1.0 / log_rate
        self._last_log_time = self.get_clock().now()

        topic = f'/{robot_name}/pose'
        self.subscription = self.create_subscription(
            Pose,
            topic,
            self.pose_callback,
            10
        )
        self.get_logger().info(f'Listening to {topic} at {log_rate} Hz ...')

    def pose_callback(self, msg: Pose):
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds * 1e-9

        if elapsed < self._log_interval:
            return

        self._last_log_time = now
        self.get_logger().info(
            f'Position → x: {msg.position.x:.3f}, '
            f'y: {msg.position.y:.3f}, '
            f'z: {msg.position.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()