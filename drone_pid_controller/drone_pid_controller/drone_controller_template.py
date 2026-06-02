import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult

from drone_pid_controller.pid import PID
from drone_pid_controller.trajectory import get_target

from scipy.spatial.transform import Rotation as R


def quaternion_to_euler_scipy(w, x, y, z):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    Returned in degrees.
    """
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=True)


def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


class QuadcopterPID(Node):

    def __init__(self):
        super().__init__('quadcopter_pid')

        # =====================================================
        # Parameters
        # =====================================================

        self.initialize_parameters()

        self.add_on_set_parameters_callback(
            self.param_callback
        )

        # =====================================================
        # Publishers
        # =====================================================

        self.path_pub = self.create_publisher(
            Path,
            '/drone_path',
            10
        )

        self.motor_pub = self.create_publisher(
            Actuators,
            '/X3/gazebo/command/motor_speed',
            10
        )

        # =====================================================
        # Subscribers
        # =====================================================

        self.pose_sub = self.create_subscription(
            PoseArray,
            '/world/quadcopter/pose/info',
            self.pose_callback,
            10
        )

        # =====================================================
        # Path message
        # =====================================================

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        # =====================================================
        # Drone state
        # =====================================================

        self.current_pose = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # =====================================================
        # PID Controllers
        # =====================================================

        # TODO:
        # Create PID controllers for:
        # 1. X position
        # 2. Y position
        # 3. Z position
        # 4. Roll stabilization
        # 5. Pitch stabilization
        # 6. Yaw stabilization

        # Example:

        # self.pid_x = PID(
        #     kp=80.0,
        #     ki=1.0,
        #     kd=50.0,
        #     output_limit=5.0,
        #     integral_limit=50.0
        # )

        # self.pid_roll = PID(
        #     kp=0.1,
        #     ki=0.0,
        #     kd=1.0,
        #     output_limit=100.0,
        #     integral_limit=10.0
        # )

        # =====================================================
        # Timing
        # =====================================================

        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        # =====================================================
        # Trajectory
        # =====================================================

        self.traj_type = self.get_parameter(
            "trajectory"
        ).value

        # =====================================================
        # Timer
        # =====================================================

        self.create_timer(
            0.02,
            self.control_loop
        )

    # =========================================================
    # Pose Callback
    # =========================================================

    def pose_callback(self, msg: PoseArray):

        if len(msg.poses) <= 1:
            self.get_logger().warn(
                "PoseArray has fewer than 2 poses"
            )
            return

        self.current_pose = msg.poses[1]

        self.current_x = self.current_pose.position.x
        self.current_y = self.current_pose.position.y
        self.current_z = self.current_pose.position.z

        self.roll, self.pitch, self.yaw = quaternion_to_euler_scipy(
            self.current_pose.orientation.w,
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z
        )

        # =====================================================
        # Publish path
        # =====================================================

        pose = PoseStamped()

        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'world'
        pose.pose = self.current_pose

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)

        self.path_pub.publish(self.path_msg)

    # =========================================================
    # Main Control Loop
    # =========================================================

    def control_loop(self):

        if self.current_pose is None:
            return

        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0.001:
            return

        self.last_time = now

        t = (now - self.start_time).nanoseconds / 1e9

        # =====================================================
        # Get trajectory target
        # =====================================================

        params = self.get_trajectory_params()

        tx, ty, tz = get_target(
            self.traj_type,
            params,
            t
        )

        # =====================================================
        # TODO:
        # Compute position errors
        #
        # Formula:
        # error = target - current
        #
        # Example:
        # err_x = tx - self.current_x
        # err_y = ty - self.current_y
        # err_z = tz - self.current_z
        # =====================================================

        # =====================================================
        # TODO:
        # Implement altitude PID controller
        #
        # Use the Z-axis PID controller to compute
        # vertical motor correction.
        #
        # Example:
        # omega_z = self.pid_z.compute(
        #     err_z,
        #     dt
        # )
        # =====================================================

        # =====================================================
        # TODO:
        # Implement position controller
        #
        # Convert X and Y position errors into
        # desired roll and pitch angles.
        #
        # Example:
        # desired_roll = self.pid_y.compute(
        #     err_y,
        #     dt
        # )
        #
        # desired_pitch = self.pid_x.compute(
        #     err_x,
        #     dt
        # )
        # =====================================================

        # =====================================================
        # TODO:
        # Clamp desired roll and pitch angles
        #
        # Example:
        # desired_roll = clamp(
        #     desired_roll,
        #     -5.0,
        #     5.0
        # )
        #
        # desired_pitch = clamp(
        #     desired_pitch,
        #     -5.0,
        #     5.0
        # )
        # =====================================================

        # =====================================================
        # TODO:
        # Compute roll and pitch errors
        #
        # Example:
        # roll_error = -desired_roll - self.roll
        # pitch_error = desired_pitch - self.pitch
        # =====================================================

        # =====================================================
        # TODO:
        # Implement attitude PID controllers
        #
        # Example:
        # omega_roll = self.pid_roll.compute(
        #     roll_error,
        #     dt
        # )
        #
        # omega_pitch = self.pid_pitch.compute(
        #     pitch_error,
        #     dt
        # )
        # =====================================================

        # =====================================================
        # TODO:
        # Define base motor speed
        #
        # Example:
        # base_speed = 636.0
        # =====================================================

        # =====================================================
        # TODO:
        # Implement motor mixing equations
        #
        # Example:
        #
        # motor0 = clamp(
        #     base_speed - omega_roll - omega_pitch + omega_z,
        #     400.0,
        #     800.0
        # )
        #
        # motor1 = clamp(
        #     base_speed + omega_roll + omega_pitch + omega_z,
        #     400.0,
        #     800.0
        # )
        #
        # motor2 = clamp(
        #     base_speed + omega_roll - omega_pitch + omega_z,
        #     400.0,
        #     800.0
        # )
        #
        # motor3 = clamp(
        #     base_speed - omega_roll + omega_pitch + omega_z,
        #     400.0,
        #     800.0
        # )
        # =====================================================

        # =====================================================
        # Publish actuator command
        # =====================================================

        cmd = Actuators()

        cmd.header = Header()
        cmd.header.stamp = now.to_msg()

        # =====================================================
        # TODO:
        # Assign motor speeds to cmd.velocity
        #
        # Example:
        # cmd.velocity = [
        #     motor0,
        #     motor1,
        #     motor2,
        #     motor3
        # ]
        # =====================================================

        self.motor_pub.publish(cmd)

        # =====================================================
        # TODO:
        # Add logging information
        #
        # Example:
        #
        # self.get_logger().info(
        #     f"Target=({tx:.2f}, {ty:.2f}, {tz:.2f}) "
        #     f"Position=({self.current_x:.2f}, "
        #     f"{self.current_y:.2f}, "
        #     f"{self.current_z:.2f})"
        # )
        # =====================================================

    # =========================================================
    # Parameters
    # =========================================================

    def initialize_parameters(self):

        self.declare_parameter(
            "trajectory",
            "figure8"
        )

        # Figure 8
        self.declare_parameter(
            "A",
            2.0
        )

        self.declare_parameter(
            "w",
            0.3
        )

        self.declare_parameter(
            "height",
            1.0
        )

        # Spiral
        self.declare_parameter(
            "radius",
            2.0
        )

        self.declare_parameter(
            "climb_rate",
            0.1
        )

        # Square
        self.declare_parameter(
            "side",
            4.0
        )

        self.declare_parameter(
            "speed",
            1.0
        )

    # =========================================================
    # Dynamic parameter updates
    # =========================================================

    def param_callback(self, params):

        for p in params:

            if p.name == "trajectory":

                self.traj_type = p.value

                self.get_logger().info(
                    f"Switched trajectory -> {self.traj_type}"
                )

        return SetParametersResult(
            successful=True
        )

    # =========================================================
    # Trajectory parameter getter
    # =========================================================

    def get_trajectory_params(self):

        if self.traj_type == "figure8":

            return {
                "A": self.get_parameter("A").value,
                "w": self.get_parameter("w").value,
                "height": self.get_parameter("height").value
            }

        elif self.traj_type == "circle":

            return {
                "radius": self.get_parameter("radius").value,
                "w": self.get_parameter("w").value,
                "height": self.get_parameter("height").value
            }

        elif self.traj_type == "spiral":

            return {
                "radius": self.get_parameter("radius").value,
                "climb_rate": self.get_parameter("climb_rate").value
            }

        elif self.traj_type == "square":

            return {
                "side": self.get_parameter("side").value,
                "speed": self.get_parameter("speed").value
            }

        return {}


# =============================================================
# Main
# =============================================================

def main():

    rclpy.init()

    node = QuadcopterPID()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()