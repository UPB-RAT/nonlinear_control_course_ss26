import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult

from drone_pid_controller.trajectory import get_target
from scipy.spatial.transform import Rotation as R

def quaternion_to_euler_scipy(w, x, y, z):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    Returned in radians for control math.
    """
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=False)

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class SMC:
    """
    Sliding Mode Controller using a boundary layer (tanh) to prevent chattering.
    """
    def __init__(self, lambda_gain, k_gain, phi_boundary=0.1, output_limit=None):
        self.lambda_gain = lambda_gain  # Slope of sliding surface
        self.k_gain = k_gain            # Robust switching gain
        self.phi_boundary = phi_boundary # Boundary layer thickness
        self.output_limit = output_limit
        self.last_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
            
        # Calculate error derivative
        error_dot = (error - self.last_error) / dt
        self.last_error = error

        # Define sliding surface s = error_dot + lambda * error
        s = error_dot + self.lambda_gain * error

        # Control law: Equivalent + Switching (using tanh to avoid chattering)
        v = self.lambda_gain * error_dot + self.k_gain * np.tanh(s / self.phi_boundary)

        if self.output_limit is not None:
            v = clamp(v, -self.output_limit, self.output_limit)
            
        return v

class QuadcopterSMC(Node):

    def __init__(self):
        super().__init__('quadcopter_smc')

        # =====================================================
        # Quadcopter Physical Parameters (Gazebo X3)
        # =====================================================
        self.mass = 1.5           # kg 
        self.gravity = 9.81       # m/s^2
        self.Ixx = 0.0347563      # kg*m^2
        self.Iyy = 0.07           # kg*m^2
        self.Izz = 0.0977         # kg*m^2
        
        self.L_x = 0.13           # Pitch leverage
        self.L_y = 0.21           # Roll leverage
        
        self.kf = 8.54858e-06     # Thrust coefficient
        self.km = 0.016 * self.kf # Torque coefficient

        # =====================================================
        # Parameters & Setup
        # =====================================================
        self.initialize_parameters()
        self.add_on_set_parameters_callback(self.param_callback)

        self.path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.motor_pub = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)

        self.pose_sub = self.create_subscription(PoseArray, '/world/quadcopter/pose/info', self.pose_callback, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        # Drone state
        self.current_x = self.current_y = self.current_z = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.last_roll = self.last_pitch = self.last_yaw = 0.0

        # =====================================================
        # Pure Sliding Mode Controllers
        # =====================================================
        # Position controllers (Outer Loop)
        self.smc_x = SMC(lambda_gain=1.5, k_gain=1.0, phi_boundary=0.5, output_limit=3.0)
        self.smc_y = SMC(lambda_gain=1.5, k_gain=1.0, phi_boundary=0.5, output_limit=3.0)
        self.smc_z = SMC(lambda_gain=2.5, k_gain=2.0, phi_boundary=0.5, output_limit=8.0)

        # Attitude controllers (Inner Loop)
        # K gains increased slightly to suppress unmodeled gyroscopic forces
        self.smc_roll = SMC(lambda_gain=6.0, k_gain=8.0, phi_boundary=0.5, output_limit=20.0)
        self.smc_pitch = SMC(lambda_gain=6.0, k_gain=8.0, phi_boundary=0.5, output_limit=20.0)
        self.smc_yaw = SMC(lambda_gain=3.0, k_gain=4.0, phi_boundary=0.5, output_limit=10.0)

        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.traj_type = self.get_parameter("trajectory").value

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) <= 1:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        if dt <= 0.005: 
            return

        self.last_time = now
        t = (now - self.start_time).nanoseconds / 1e9

        current_pose = msg.poses[1]

        self.current_x = current_pose.position.x
        self.current_y = current_pose.position.y
        self.current_z = current_pose.position.z

        self.roll, self.pitch, self.yaw = quaternion_to_euler_scipy(
            current_pose.orientation.w,
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z
        )

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'world'
        pose.pose = current_pose
        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        
        # =====================================================
        # Prevent ROS 2 Memory & DDS Choke
        # =====================================================
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses = self.path_msg.poses[-5000:]
            
        self.path_pub.publish(self.path_msg)

        self.control_step(t, dt)

    def control_step(self, t, dt):
        # We no longer explicitly need p, q, r for the pure SMC math, 
        # but tracking them is useful if you want to log rotational speeds later.
        self.last_roll = self.roll
        self.last_pitch = self.pitch
        self.last_yaw = self.yaw

        # Trajectory
        params = self.get_trajectory_params()
        tx, ty, tz = get_target(self.traj_type, params, t)

        err_x = tx - self.current_x
        err_y = ty - self.current_y
        err_z = tz - self.current_z

        # =====================================================
        # 1. Outer Loop Z: Altitude SMC
        # =====================================================
        v_z = self.smc_z.compute(err_z, dt)
        
        cos_roll_pitch = max(np.cos(self.roll) * np.cos(self.pitch), 0.1) 
        U1 = (self.mass / cos_roll_pitch) * (v_z + self.gravity)
        U1 = clamp(U1, self.mass * self.gravity * 0.5, self.mass * self.gravity * 2.0)

        # =====================================================
        # 2. Outer Loop XY: Position to Attitude Mapping
        # =====================================================
        # Clamped safely to avoid extreme tilt demands
        v_x = clamp(self.smc_x.compute(err_x, dt), -2.5, 2.5)
        v_y = clamp(self.smc_y.compute(err_y, dt), -2.5, 2.5)
        
        sin_yaw = np.sin(self.yaw)
        cos_yaw = np.cos(self.yaw)

        term_phi = (self.mass / U1) * (v_x * sin_yaw - v_y * cos_yaw)
        desired_roll = np.arcsin(clamp(term_phi, -0.3, 0.3))

        term_theta = (self.mass / (U1 * np.cos(desired_roll))) * (v_x * cos_yaw + v_y * sin_yaw)
        desired_pitch = np.arcsin(clamp(term_theta, -0.3, 0.3))

        # =====================================================
        # 3. Inner Loop: Pure Attitude SMC
        # =====================================================
        err_roll = (desired_roll - self.roll + np.pi) % (2 * np.pi) - np.pi
        err_pitch = (desired_pitch - self.pitch + np.pi) % (2 * np.pi) - np.pi
        err_yaw = (0.0 - self.yaw + np.pi) % (2 * np.pi) - np.pi

        v_phi = self.smc_roll.compute(err_roll, dt)
        v_theta = self.smc_pitch.compute(err_pitch, dt)
        v_psi = self.smc_yaw.compute(err_yaw, dt)

        # Pure SMC: Treating nonlinear cross-coupling as bounded disturbances
        # The SMC robust switching gain handles the unmodeled dynamics
        U2 = self.Ixx * v_phi
        U3 = self.Iyy * v_theta
        U4 = self.Izz * v_psi

        # =====================================================
        # 4. Control Allocation (Motor Mixing)
        # =====================================================
        t_base = U1 / (4 * self.kf)
        t_roll = U2 / (4 * self.kf * self.L_y)  
        t_pitch = U3 / (4 * self.kf * self.L_x) 
        t_yaw = U4 / (4 * self.km)

        w0_sq = t_base - t_roll - t_pitch - t_yaw
        w1_sq = t_base + t_roll + t_pitch - t_yaw
        w2_sq = t_base + t_roll - t_pitch + t_yaw
        w3_sq = t_base - t_roll + t_pitch + t_yaw

        motor0 = clamp(np.sqrt(max(w0_sq, 0.0)), 200.0, 1200.0)
        motor1 = clamp(np.sqrt(max(w1_sq, 0.0)), 200.0, 1200.0)
        motor2 = clamp(np.sqrt(max(w2_sq, 0.0)), 200.0, 1200.0)
        motor3 = clamp(np.sqrt(max(w3_sq, 0.0)), 200.0, 1200.0)

        cmd = Actuators()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.velocity = [motor0, motor1, motor2, motor3]
        self.motor_pub.publish(cmd)

        self.get_logger().info(
            f"[Pure SMC {self.traj_type}] "
            f"E=({err_x:.2f}, {err_y:.2f}, {err_z:.2f}) "
            f"M=({motor0:.0f}, {motor1:.0f}, {motor2:.0f}, {motor3:.0f})",
            throttle_duration_sec=0.5
        )

    # =========================================================
    # Parameters Logic 
    # =========================================================
    def initialize_parameters(self):
        self.declare_parameter("trajectory", "circle")
        self.declare_parameter("A", 2.0)
        self.declare_parameter("w", 0.3)
        self.declare_parameter("height", 1.0)
        self.declare_parameter("radius", 2.0)
        self.declare_parameter("climb_rate", 0.1)
        self.declare_parameter("side", 4.0)
        self.declare_parameter("speed", 1.0)

    def param_callback(self, params):
        for p in params:
            if p.name == "trajectory":
                self.traj_type = p.value
        return SetParametersResult(successful=True)

    def get_trajectory_params(self):
        if self.traj_type == "figure8": return {"A": self.get_parameter("A").value, "w": self.get_parameter("w").value, "height": self.get_parameter("height").value}
        elif self.traj_type == "circle": return {"radius": self.get_parameter("radius").value, "w": self.get_parameter("w").value, "height": self.get_parameter("height").value}
        elif self.traj_type == "spiral": return {"radius": self.get_parameter("radius").value,"w": self.get_parameter("w").value, "climb_rate": self.get_parameter("climb_rate").value}
        elif self.traj_type == "square": return {"side": self.get_parameter("side").value, "speed": self.get_parameter("speed").value}
        return {}

def main():
    rclpy.init()
    node = QuadcopterSMC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()