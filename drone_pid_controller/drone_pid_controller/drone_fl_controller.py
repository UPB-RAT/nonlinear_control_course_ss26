import rclpy
from rclpy.node import Node
import numpy as np

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
    Returned in radians for Feedback Linearization math.
    """
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=False)

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class QuadcopterFeedbackLinearization(Node):

    def __init__(self):
        super().__init__('quadcopter_feedback_linearization')

        # =====================================================
        # Quadcopter Physical Parameters
        # (Matched exactly to Gazebo X3 SDF structure)
        # =====================================================
        self.mass = 1.5           # kg 
        self.gravity = 9.81       # m/s^2
        self.Ixx = 0.0347563      # kg*m^2
        self.Iyy = 0.07           # kg*m^2
        self.Izz = 0.0977         # kg*m^2
        
        # Exact lever arms from the SDF file (Rectangular frame)
        self.L_x = 0.13           # distance from center to front/back rotors (Pitch leverage)
        self.L_y = 0.21           # distance from center to left/right rotors (Roll leverage)
        
        self.kf = 8.54858e-06     # Thrust coefficient
        self.km = 0.016 * self.kf # Torque coefficient

        # =====================================================
        # Parameters & Setup
        # =====================================================
        self.initialize_parameters()
        self.add_on_set_parameters_callback(self.param_callback)

        self.path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.motor_pub = self.create_publisher(Actuators, '/X3/gazebo/command/motor_speed', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseArray, '/world/quadcopter/pose/info', self.pose_callback, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'

        # =====================================================
        # Drone state
        # =====================================================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.last_yaw = 0.0

        # =====================================================
        # PID Controllers (Outputs are Virtual Accelerations)
        # =====================================================
        # Position controllers (PD only to prevent integral windup blowing up the math)
        self.pid_x = PID(kp=1.5, ki=0.0, kd=2.5, output_limit=5.0, integral_limit=0.0)
        self.pid_y = PID(kp=1.5, ki=0.0, kd=2.5, output_limit=5.0, integral_limit=0.0)
        self.pid_z = PID(kp=4.0, ki=0.0, kd=5.0, output_limit=10.0, integral_limit=0.0)

        # Attitude controllers 
        self.pid_roll = PID(kp=8.0, ki=0.0, kd=3.0, output_limit=50.0, integral_limit=0.0)
        self.pid_pitch = PID(kp=8.0, ki=0.0, kd=3.0, output_limit=50.0, integral_limit=0.0)
        self.pid_yaw = PID(kp=4.0, ki=0.0, kd=1.0, output_limit=20.0, integral_limit=0.0)

        # Timing
        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()
        self.traj_type = self.get_parameter("trajectory").value

        # Removed the independent Timer. Control is now perfectly synced to Gazebo pose updates.

    def pose_callback(self, msg: PoseArray):
        """
        Moving the control loop inside the callback ensures dt is exact 
        and eliminates finite-difference noise spikes.
        """
        if len(msg.poses) <= 1:
            self.get_logger().warn("PoseArray has fewer than 2 poses")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9

        # Protect against duplicate messages in the same tick or zero division
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

        # Publish path
        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'world'
        pose.pose = current_pose
        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        # Execute control step synchronously
        self.control_step(t, dt)

    def control_step(self, t, dt):
        # =====================================================
        # Estimate Angular Rates (p, q, r)
        # =====================================================
        p = (self.roll - self.last_roll) / dt
        q = (self.pitch - self.last_pitch) / dt
        r = (self.yaw - self.last_yaw) / dt
        
        self.last_roll = self.roll
        self.last_pitch = self.pitch
        self.last_yaw = self.yaw

        # =====================================================
        # Trajectory Target & Position Errors
        # =====================================================
        params = self.get_trajectory_params()
        tx, ty, tz = get_target(self.traj_type, params, t)

        err_x = tx - self.current_x
        err_y = ty - self.current_y
        err_z = tz - self.current_z

        # =====================================================
        # 1. Inner Loop (Altitude Linearization)
        # =====================================================
        v_z = self.pid_z.compute(err_z, dt)
        
        # Calculate Physical Thrust (U1)
        # Prevent division by zero if it tilts past 90 deg
        cos_roll_pitch = max(np.cos(self.roll) * np.cos(self.pitch), 0.1) 
        U1 = (self.mass / cos_roll_pitch) * (v_z + self.gravity)
        
        # CRITICAL: Prevent U1 from getting too small (loss of attitude control) 
        # or too large (violent jumps)
        U1 = clamp(U1, self.mass * self.gravity * 0.5, self.mass * self.gravity * 2.0)

        # =====================================================
        # 2. Outer Loop (Position to Desired Attitude)
        # =====================================================
        v_x = self.pid_x.compute(err_x, dt)
        v_y = self.pid_y.compute(err_y, dt)
        
        # Clamp virtual accelerations to physically reasonable limits
        v_x = clamp(v_x, -4.0, 4.0)
        v_y = clamp(v_y, -4.0, 4.0)
        
        sin_yaw = np.sin(self.yaw)
        cos_yaw = np.cos(self.yaw)

        # Desired Roll (phi_d)
        term_phi = (self.mass / U1) * (v_x * sin_yaw - v_y * cos_yaw)
        desired_roll = np.arcsin(clamp(term_phi, -0.3, 0.3))

        # Desired Pitch (theta_d)
        term_theta = (self.mass / (U1 * np.cos(desired_roll))) * (v_x * cos_yaw + v_y * sin_yaw)
        desired_pitch = np.arcsin(clamp(term_theta, -0.3, 0.3))

        # =====================================================
        # 3. Inner Loop (Attitude Linearization)
        # =====================================================
        err_roll = (desired_roll - self.roll + np.pi) % (2 * np.pi) - np.pi
        err_pitch = (desired_pitch - self.pitch + np.pi) % (2 * np.pi) - np.pi
        err_yaw = (0.0 - self.yaw + np.pi) % (2 * np.pi) - np.pi

        v_phi = self.pid_roll.compute(err_roll, dt)
        v_theta = self.pid_pitch.compute(err_pitch, dt)
        v_psi = self.pid_yaw.compute(err_yaw, dt)

        # Algebraic Cancellation of Nonlinear Dynamics (U2, U3, U4)
        U2 = self.Ixx * (v_phi - ((self.Iyy - self.Izz) / self.Ixx) * q * r)
        U3 = self.Iyy * (v_theta - ((self.Izz - self.Ixx) / self.Iyy) * p * r)
        U4 = self.Izz * (v_psi - ((self.Ixx - self.Iyy) / self.Izz) * p * q)

        # =====================================================
        # 4. Control Allocation (Motor Mixing)
        # =====================================================
        # Based on X3 SDF Coordinates (Rectangular Frame):
        # rotor_0: Front-Right, CCW
        # rotor_1: Back-Left, CCW
        # rotor_2: Front-Left, CW
        # rotor_3: Back-Right, CW

        t_base = U1 / (4 * self.kf)
        t_roll = U2 / (4 * self.kf * self.L_y)  # Uses L_y for Roll Leverage
        t_pitch = U3 / (4 * self.kf * self.L_x) # Uses L_x for Pitch Leverage
        t_yaw = U4 / (4 * self.km)

        # rotor_0: Front-Right, CCW (-Roll, -Pitch, -Yaw)
        w0_sq = t_base - t_roll - t_pitch - t_yaw

        # rotor_1: Back-Left, CCW (+Roll, +Pitch, -Yaw)
        w1_sq = t_base + t_roll + t_pitch - t_yaw

        # rotor_2: Front-Left, CW (+Roll, -Pitch, +Yaw)
        w2_sq = t_base + t_roll - t_pitch + t_yaw

        # rotor_3: Back-Right, CW (-Roll, +Pitch, +Yaw)
        w3_sq = t_base - t_roll + t_pitch + t_yaw

        # Compute motor commands, clamping to avoid imaginary roots
        motor0 = np.sqrt(max(w0_sq, 0.0))
        motor1 = np.sqrt(max(w1_sq, 0.0))
        motor2 = np.sqrt(max(w2_sq, 0.0))
        motor3 = np.sqrt(max(w3_sq, 0.0))

        # Increased limits to allow for aggressive maneuvering and prevent clipping
        motor0 = clamp(motor0, 200.0, 1200.0)
        motor1 = clamp(motor1, 200.0, 1200.0)
        motor2 = clamp(motor2, 200.0, 1200.0)
        motor3 = clamp(motor3, 200.0, 1200.0)

        # Publish Command
        cmd = Actuators()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.velocity = [motor0, motor1, motor2, motor3]
        self.motor_pub.publish(cmd)

        # Logging (Converting back to degrees for readability)
        self.get_logger().info(
            f"[{self.traj_type}] "
            f"T=({tx:.2f}, {ty:.2f}, {tz:.2f}) "
            f"P=({self.current_x:.2f}, {self.current_y:.2f}, {self.current_z:.2f}) "
            f"RP_deg=({np.degrees(self.roll):.1f}, {np.degrees(self.pitch):.1f}) "
            f"DesRP_deg=({np.degrees(desired_roll):.1f}, {np.degrees(desired_pitch):.1f}) "
            f"M=({motor0:.1f}, {motor1:.1f}, {motor2:.1f}, {motor3:.1f})",
            throttle_duration_sec=0.5
        )

    # =========================================================
    # Parameters Logic (Unchanged)
    # =========================================================
    def initialize_parameters(self):
        self.declare_parameter("trajectory", "spiral")
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
                self.get_logger().info(f"Switched trajectory -> {self.traj_type}")
        return SetParametersResult(successful=True)

    def get_trajectory_params(self):
        if self.traj_type == "figure8":
            return {"A": self.get_parameter("A").value, "w": self.get_parameter("w").value, "height": self.get_parameter("height").value}
        elif self.traj_type == "circle":
            return {"radius": self.get_parameter("radius").value, "w": self.get_parameter("w").value, "height": self.get_parameter("height").value}
        elif self.traj_type == "spiral":
            return {"radius": self.get_parameter("radius").value,"w": self.get_parameter("w").value, "climb_rate": self.get_parameter("climb_rate").value}
        elif self.traj_type == "square":
            return {"side": self.get_parameter("side").value, "speed": self.get_parameter("speed").value}
        return {}

def main():
    rclpy.init()
    node = QuadcopterFeedbackLinearization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()