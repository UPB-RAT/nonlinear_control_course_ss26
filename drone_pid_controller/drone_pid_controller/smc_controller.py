"""
================================================================================
SMC CONTROLLER FOR QUADCOPTER - VARIANT 3: MONOLITHIC 6-DOF MIMO SMC
================================================================================

TEACHING LEVEL : Advanced (Step 3 in the SMC lecture series)
LAUNCH FILE    : smc_controller.launch.py
ENTRY POINT    : SMC_controller_node

--------------------------------------------------------------------------------
WHAT IS THIS SCRIPT?
--------------------------------------------------------------------------------
This is the MOST ADVANCED of the three SMC implementations. It abandons the
explicit outer-loop / inner-loop decomposition and builds ONE controller
that acts on the full 6-dimensional state vector:

    e = [e_x, e_y, e_z, e_phi, e_theta, e_psi]^T

A single MIMO_SMC block returns a 6-dimensional virtual-acceleration
vector nu. A control-effectiveness matrix G(X) then maps nu to the four
physical control inputs (U1, U2, U3, U4) by means of a damped pseudo-inverse.

--------------------------------------------------------------------------------
WHY DO WE NEED A PSEUDO-INVERSE?
--------------------------------------------------------------------------------
The drone is UNDERACTUATED: 6 states, 4 actuators. A 6x4 matrix G(X) maps
control inputs to state accelerations. We cannot invert a non-square
matrix, so we use a damped Moore-Penrose pseudo-inverse
    G_pinv = (G^T G + lambda_damp * I)^-1 G^T
which gives the LEAST-SQUARES best-fit physical input that achieves the
desired virtual acceleration. The lambda_damp term regularises the
solution and prevents singularities at hover (cos(roll)=cos(pitch)=1).

--------------------------------------------------------------------------------
WHAT DOES THIS SCRIPT TEACH?
--------------------------------------------------------------------------------
- A 6-DOF sliding manifold that captures ALL states at once.
- The role of f(X), the nonlinear drift term (here: gravity on z).
- The role of G(X), the control effectiveness matrix.
- Damped pseudo-inverse allocation from virtual accelerations to
  physical inputs.
- Handling of the underactuated singularity at hover.

--------------------------------------------------------------------------------
WHEN SHOULD STUDENTS STUDY THIS SCRIPT?
--------------------------------------------------------------------------------
Only AFTER they have understood:
1. The scalar SMC (smc_ind_controller.py)
2. The vector / MIMO SMC (smc_mimo.py)
3. Why the cascade is needed and what the geometry does.

--------------------------------------------------------------------------------
STUDENT EXERCISES
--------------------------------------------------------------------------------
1. Toggle lambda_damp between 0.0 and 0.1 and observe the motor commands.
2. Move from a diagonal Lambda to a full Lambda and study cross-coupling.
3. Disable the pseudo-inverse damping and let the drone hover: the
   allocation becomes ill-conditioned because all four motors are at
   almost the same speed.
================================================================================
"""

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
    True Multiple-Input Multiple-Output (MIMO) Sliding Mode Controller.
    Computes a full virtual control vector using multi-dimensional manifolds.
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
        if dt <= 0.0:
            return np.zeros((self.num_dims, 1))
            
        error = np.array(error_vector).reshape(self.num_dims, 1)
        
        # 1. Error derivative vector (safely handling angular wraparound)
        error_diff = error - self.last_error
        
        for idx in self.angle_indices:
            # Force the angular difference to be between -pi and pi
            error_diff[idx] = (error_diff[idx] + np.pi) % (2 * np.pi) - np.pi
            
        error_dot = error_diff / dt
        self.last_error = error

        # 2. Multi-dimensional Sliding Manifold: S = e_dot + Lambda * e
        S = error_dot + self.Lambda @ error

        # 3. Vector norm boundary layer (Prevents chattering across all axes simultaneously)
        s_norm = np.linalg.norm(S)
        switching_term = S / (s_norm + self.delta)

        # 4. Virtual Control Vector
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

        # =====================================================
        # MONOLITHIC 6-DOF MIMO SMC
        # ---------------------------------------------------------
        # One controller for the FULL state vector
        #     e = [x, y, z, roll, pitch, yaw]^T
        # The Lambda and K matrices are diagonal for clarity.
        # Indices [3, 4, 5] are flagged as angles so that the
        # angular wrap-around is handled before differentiation.
        # =====================================================
        lambda_diag = [1.5, 1.5, 2.5, 6.0, 6.0, 3.0]
        k_diag      = [0.1, 0.1, 2.0, 8.0, 8.0, 4.0]

        self.mimo_smc = MIMO_SMC(
            num_dims=6,
            lambda_matrix=np.diag(lambda_diag),
            k_matrix=np.diag(k_diag),
            delta=1.5,
            angle_indices=[3, 4, 5]  # Roll, Pitch, Yaw are angles
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
        # Prevent ROS 2 Memory & DDS Choke
        # =====================================================
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses = self.path_msg.poses[-5000:]
            
        self.path_pub.publish(self.path_msg)

        self.control_step(t, dt)

    def control_step(self, t, dt):
        # 1. Fetch Trajectory Target
        params = self.get_trajectory_params()

        # Prevent floating-point precision loss
        if "w" in params and params["w"] > 0:
            bounded_t = t % ((2 * np.pi) / params["w"])
        else:
            bounded_t = t

        tx, ty, tz = get_target(self.traj_type, params, bounded_t)

        err_x = tx - self.current_x
        err_y = ty - self.current_y
        err_z = tz - self.current_z

        # 2. Heuristic Target Tilt (Prevents Hover Singularity in 6-DOF matrix)
        # The monolithic controller would otherwise need an inverse dynamics
        # step to obtain a desired attitude from the position error. To
        # keep the script simple, we propose a desired roll/pitch that
        # points the drone's thrust vector towards the position target.
        k_tilt = 0.25
        desired_roll  = clamp(k_tilt * (err_x * np.sin(self.yaw) - err_y * np.cos(self.yaw)), -0.35, 0.35)
        desired_pitch = clamp(k_tilt * (err_x * np.cos(self.yaw) + err_y * np.sin(self.yaw)), -0.35, 0.35)
        desired_yaw   = 0.0

        err_roll  = (desired_roll  - self.roll  + np.pi) % (2 * np.pi) - np.pi
        err_pitch = (desired_pitch - self.pitch + np.pi) % (2 * np.pi) - np.pi
        err_yaw   = (desired_yaw   - self.yaw   + np.pi) % (2 * np.pi) - np.pi

        # 3. Construct the 6-Dimensional Error Vector
        error_vec = np.array([err_x, err_y, err_z, err_roll, err_pitch, err_yaw])

        # 4. Compute the Virtual Robust Acceleration Vector via MIMO SMC
        v_vector = self.mimo_smc.compute(error_vec, dt)

        # 5. Define the Nonlinear Drift Dynamics f(X) (Cleaned from gyro noise)
        # The drift term contains all the dynamics that the SMC does NOT
        # need to overcome with its switching gain. For a quadcopter in
        # hover, the dominant drift is gravity on the z-axis.
        f_x = np.zeros((6, 1))
        f_x[2, 0] = -self.gravity

        # 6. Define the Control Matrix G(X) mapping the 4 inputs to 6 states
        # Rows correspond to [x, y, z, roll, pitch, yaw].
        # Columns correspond to [U1 (thrust), U2 (roll torque), U3 (pitch torque), U4 (yaw torque)].
        phi, theta, psi = self.roll, self.pitch, self.yaw
        G_x = np.zeros((6, 4))

        # True physical mapping
        gx_true = (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi)) / self.mass
        gy_true = (np.cos(phi) * np.sin(theta) * np.sin(psi) - np.sin(phi) * np.cos(psi)) / self.mass

        # Tiny epsilon tolerance to prevent hover singularity
        epsilon = 1e-4
        G_x[0, 0] = gx_true if abs(gx_true) > epsilon else np.sign(gx_true + 1e-9) * epsilon
        G_x[1, 0] = gy_true if abs(gy_true) > epsilon else np.sign(gy_true + 1e-9) * epsilon

        G_x[2, 0] = (np.cos(phi) * np.cos(theta)) / self.mass
        G_x[3, 1] = 1.0 / self.Ixx
        G_x[4, 2] = 1.0 / self.Iyy
        G_x[5, 3] = 1.0 / self.Izz

        # 7. Damped Pseudo-Inverse Control Allocation (Tikhonov Regularization)
        # G is 6x4 (more rows than columns). The damped pseudo-inverse
        #     G_pinv = (G^T G + lambda_damp * I)^-1 G^T
        # gives the LEAST-SQUARES best fit of the four physical inputs
        # to the six desired virtual accelerations. The damping term
        # lambda_damp * I prevents the inverse from blowing up when G
        # is ill-conditioned (e.g. at perfect hover).
        lambda_damp = 0.01
        G_pinv = np.linalg.inv(G_x.T @ G_x + lambda_damp * np.eye(4)) @ G_x.T
        U_physical = G_pinv @ (v_vector - f_x)

        U1 = float(U_physical[0, 0])
        U2 = float(U_physical[1, 0])
        U3 = float(U_physical[2, 0])
        U4 = float(U_physical[3, 0])

        # Safely clamp U1 to prevent free-fall
        U1 = clamp(U1, self.mass * self.gravity * 0.5, self.mass * self.gravity * 2.0)

        # 8. Physical Motor Mixing
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
            f"[Monolithic SMC 6-DOF {self.traj_type}] "
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