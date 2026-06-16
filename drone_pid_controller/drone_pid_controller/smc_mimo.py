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

class MIMO_SMC:
    """
    A vector (multi-dimensional) Sliding Mode Controller.

    Where the scalar SMC class works on one error at a time, this class
    works on a VECTOR of tracking errors simultaneously. Internally it
    builds a single multi-dimensional sliding surface:

        S(t) = e_dot(t) + Lambda * e(t)

    and produces a vector of virtual control inputs:

        nu = Lambda * e_dot + K * S / (||S|| + delta)

    Parameters
    ----------
    num_dims       : int
        Dimension of the error vector (e.g. 3 for x/y/z, 6 for full 6-DOF).
    lambda_matrix  : (num_dims x num_dims) array
        Slope matrix of the sliding surface. Diagonal entries act as
        the per-axis lambda used in the scalar SMC. Off-diagonal entries
        introduce coupling between axes.
    k_matrix       : (num_dims x num_dims) array
        Robust switching gain matrix. Must be large enough to reject
        the worst-case disturbance on each axis.
    delta          : float
        Boundary-layer thickness for the vector norm. Analogous to phi
        in the scalar SMC.
    angle_indices  : list of int or None
        Indices of the error-vector entries that correspond to ANGLES.
        They are wrapped into [-pi, pi] before differentiation to avoid
        spurious jumps when, e.g., yaw crosses the +/-pi boundary.
    """
    def __init__(self, num_dims, lambda_matrix, k_matrix, delta=0.5, angle_indices=None):
        self.num_dims = num_dims
        self.Lambda = np.array(lambda_matrix)
        self.K = np.array(k_matrix)
        self.delta = delta
        self.last_error = np.zeros((num_dims, 1))
        # Keep track of which states are angles to prevent wraparound explosions
        self.angle_indices = angle_indices if angle_indices is not None else []

    def compute(self, error_vector, dt):
        """
        Compute one step of the vector SMC law.

        Parameters
        ----------
        error_vector : array-like of length num_dims
            Current tracking errors  e = x_desired - x_current.
        dt           : float
            Time elapsed since the previous call (seconds).

        Returns
        -------
        nu : (num_dims x 1) numpy array
            Vector of virtual control inputs (one per controlled axis).
        """
        if dt <= 0.0:
            return np.zeros((self.num_dims, 1))

        error = np.array(error_vector).reshape(self.num_dims, 1)

        # 1. Error derivative vector (safely handling angular wraparound).
        error_diff = error - self.last_error

        for idx in self.angle_indices:
            # Force the angular difference to be between -pi and pi
            error_diff[idx] = (error_diff[idx] + np.pi) % (2 * np.pi) - np.pi

        error_dot = error_diff / dt
        self.last_error = error

        # 2. Multi-dimensional Sliding Manifold:  S = e_dot + Lambda * e
        S = error_dot + self.Lambda @ error

        # 3. Vector norm boundary layer. Dividing the sliding vector by
        #    its own norm + delta yields a UNIT VECTOR switching term.
        #    This is the vector equivalent of tanh(s/phi) in the scalar case.
        s_norm = np.linalg.norm(S)
        switching_term = S / (s_norm + self.delta)

        # 4. Virtual control vector.
        nu = (self.Lambda @ error_dot) + (self.K @ switching_term)
        return nu


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
        # CASCADED MIMO SMC TUNING
        # ---------------------------------------------------------
        # Two MIMO controllers, each acting on a 3-dimensional vector.
        # The position controller drives (x, y, z). The attitude
        # controller drives (roll, pitch, yaw).
        # Diagonal Lambda/K are used for clarity, but the matrices
        # are real numpy arrays, so off-diagonal coupling is allowed.
        # =====================================================
        # 1. Outer Loop MIMO (Position X, Y, Z)
        lambda_pos = np.diag([1.5, 1.5, 2.5])
        k_pos = np.diag([0.5, 0.5, 2.0])

        self.smc_pos = MIMO_SMC(
            num_dims=3,
            lambda_matrix=lambda_pos,
            k_matrix=k_pos,
            delta=1.0,
            angle_indices=[]  # Explicitly tell the SMC that X, Y, Z are NOT angles!
        )

        # 2. Inner Loop MIMO (Attitude Roll, Pitch, Yaw)
        lambda_att = np.diag([8.0, 8.0, 4.0])
        k_att = np.diag([10.0, 10.0, 5.0])

        self.smc_att = MIMO_SMC(
            num_dims=3,
            lambda_matrix=lambda_att,
            k_matrix=k_att,
            delta=1.5,
            angle_indices=[0, 1, 2]  # Protect all 3 attitude angles from wrap-around
        )

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
        # FIX: Prevent ROS 2 Memory & DDS Choke
        # Keep only the last 1000 trajectory points 
        # =====================================================
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses = self.path_msg.poses[-1000:]
            
        self.path_pub.publish(self.path_msg)

        self.control_step(t, dt)

    def control_step(self, t, dt):
        self.last_roll = self.roll
        self.last_pitch = self.pitch
        self.last_yaw = self.yaw

        # 1. Fetch Trajectory Target
        params = self.get_trajectory_params()
        
        # Prevent floating-point precision loss for large t
        if "w" in params and params["w"] > 0:
            period = (2 * np.pi) / params["w"]
            bounded_t = t % period
        else:
            bounded_t = t
            
        tx, ty, tz = get_target(self.traj_type, params, bounded_t)

        err_x = tx - self.current_x
        err_y = ty - self.current_y
        err_z = tz - self.current_z

        # =========================================================
        # OUTER LOOP: Position MIMO SMC
        # ---------------------------------------------------------
        # The whole position error vector is processed at once,
        # producing a vector of virtual accelerations v_pos.
        # We then split v_pos into v_x, v_y, v_z for clarity, but
        # the SMC has already taken cross-axis coupling into account.
        # =========================================================
        err_pos = np.array([err_x, err_y, err_z])
        v_pos = self.smc_pos.compute(err_pos, dt)

        # Extract desired virtual accelerations (limited to prevent extreme maneuvers)
        v_x = clamp(float(v_pos[0, 0]), -3.0, 3.0)
        v_y = clamp(float(v_pos[1, 0]), -3.0, 3.0)
        v_z = float(v_pos[2, 0])

        # Geometric Mapping: Z-Acceleration to Total Thrust (U1)
        # We ensure cos_roll_pitch doesn't drop too low, preventing mathematical instability
        cos_roll_pitch = max(np.cos(self.roll) * np.cos(self.pitch), 0.5)
        U1 = (self.mass / cos_roll_pitch) * (v_z + self.gravity)
        U1 = clamp(U1, self.mass * self.gravity * 0.5, self.mass * self.gravity * 2.0)

        # Simplified Linear Mapping (Small Angle Approximation)
        desired_roll = clamp((v_x * np.sin(self.yaw) - v_y * np.cos(self.yaw)) / self.gravity, -0.35, 0.35)
        desired_pitch = clamp((v_x * np.cos(self.yaw) + v_y * np.sin(self.yaw)) / self.gravity, -0.35, 0.35)
        desired_yaw = 0.0

        # =========================================================
        # INNER LOOP: Attitude MIMO SMC
        # ---------------------------------------------------------
        # The attitude error vector is processed at once.
        # The switching matrix K is large enough to absorb the
        # gyroscopic cross-coupling terms without an explicit
        # feedback-linearization step.
        # =========================================================
        err_roll  = (desired_roll  - self.roll  + np.pi) % (2 * np.pi) - np.pi
        err_pitch = (desired_pitch - self.pitch + np.pi) % (2 * np.pi) - np.pi
        err_yaw   = (desired_yaw   - self.yaw   + np.pi) % (2 * np.pi) - np.pi

        err_att = np.array([err_roll, err_pitch, err_yaw])
        v_att = self.smc_att.compute(err_att, dt)

        # Pure Model-Free Control: Trust the K matrix to fight gyroscopic drift
        U2 = self.Ixx * float(v_att[0, 0])
        U3 = self.Iyy * float(v_att[1, 0])
        U4 = self.Izz * float(v_att[2, 0])

        # =====================================================
        # MOTOR MIXING
        # ---------------------------------------------------------
        # Same X-configuration mixer used in the scalar script.
        # Each motor command is clamped to [200, 1200] RPM.
        # =====================================================
        t_base  = U1 / (4 * self.kf)
        t_roll  = U2 / (4 * self.kf * self.L_y)
        t_pitch = U3 / (4 * self.kf * self.L_x)
        t_yaw   = U4 / (4 * self.km)

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
            f"[Cascaded MIMO SMC {self.traj_type}] "
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