"""
================================================================================
SMC CONTROLLER FOR QUADCOPTER - VARIANT 1: INDEPENDENT (SCALAR) SMC
================================================================================

TEACHING LEVEL : Beginner (Step 1 in the SMC lecture series)
LAUNCH FILE    : smc_individual_controller.launch.py
ENTRY POINT    : Cascade_SMC_ind_controller_node

--------------------------------------------------------------------------------
WHAT IS THIS SCRIPT?
--------------------------------------------------------------------------------
This is the SIMPLEST of the three SMC implementations. It is the recommended
entry point for students who are seeing Sliding Mode Control for the first time.

The core idea of SMC is to define a *sliding surface*
        s(t) = e_dot(t) + lambda * e(t)
and to design a control law that forces the system state to slide along this
surface down to the origin (e -> 0).

A continuous approximation of the theoretical sign(s) function is used,
        tanh(s / phi)
to avoid the chattering phenomenon that would otherwise destroy the motors.

--------------------------------------------------------------------------------
WHY "INDEPENDENT" / "SCALAR"?
--------------------------------------------------------------------------------
The quadcopter has 6 states (x, y, z, roll, pitch, yaw) but only 4 actuators.
We still use a CASCADED structure:

    OUTER LOOP (Position)          INNER LOOP (Attitude)
    -------------------            ---------------------
    smc_x   -> desired roll        smc_roll  -> torque U2
    smc_y   -> desired pitch       smc_pitch -> torque U3
    smc_z   -> total thrust U1     smc_yaw   -> torque U4

Notice that each of the 6 sliding surfaces is its OWN scalar SMC object.
There is no shared sliding manifold; coupling between axes (e.g. roll affects
y-position) is handled IMPLICITLY by making the switching gain K large enough
to overpower the cross-coupling terms.

--------------------------------------------------------------------------------
CONTROL LAW IMPLEMENTED
--------------------------------------------------------------------------------
For every axis i:
    error_i     = desired_i - current_i
    error_dot_i = (error_i - error_i_prev) / dt
    s_i         = error_dot_i + lambda_i * error_i
    v_i         = lambda_i * error_dot_i + K_i * tanh(s_i / phi_i)
then the virtual accelerations v_x, v_y, v_z are converted into a desired
thrust and desired roll/pitch angles, which are finally tracked by the inner
loop. The inner loop again uses scalar SMC for each attitude axis.

--------------------------------------------------------------------------------
STUDENT EXERCISES
--------------------------------------------------------------------------------
1. Change K gains and observe chattering vs. robustness trade-off.
2. Change phi and observe the boundary layer effect.
3. Add or remove the tanh (replace it with np.sign) and watch chattering appear.
4. Try aggressive trajectories (spiral) and see where each loop saturates.
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

class SMC:
    """
    A single-input / single-output (scalar) Sliding Mode Controller.

    This is the building block used by smc_ind_controller.py. One instance
    of this class controls exactly ONE state of the drone (e.g. x-position,
    roll angle, etc.). Stacking six of them together gives the full drone
    controller.

    Parameters
    ----------
    lambda_gain  : float
        Slope of the sliding surface s = e_dot + lambda * e.
        Larger lambda -> faster convergence on the surface.
    k_gain       : float
        Robust switching gain. Must be larger than the worst-case disturbance
        that the controller is expected to reject.
    phi_boundary : float
        Boundary-layer thickness. Replaces the discontinuous sign(s) with the
        smooth tanh(s/phi) to suppress chattering.
    output_limit : float or None
        Optional saturation limit on the virtual control output (e.g. max
        virtual acceleration). Protects the actuators from impossible
        commands.
    """
    def __init__(self, lambda_gain, k_gain, phi_boundary=0.1, output_limit=None):
        self.lambda_gain = lambda_gain   # Slope of sliding surface
        self.k_gain = k_gain             # Robust switching gain
        self.phi_boundary = phi_boundary # Boundary layer thickness
        self.output_limit = output_limit
        self.last_error = 0.0

    def compute(self, error, dt):
        """
        Compute one step of the scalar SMC law.

        Parameters
        ----------
        error : float
            Current tracking error  e = x_desired - x_current.
        dt    : float
            Time elapsed since the previous call (seconds).

        Returns
        -------
        v : float
            Virtual control input (e.g. virtual acceleration).
        """
        if dt <= 0.0:
            return 0.0

        # Numerical derivative of the tracking error.
        error_dot = (error - self.last_error) / dt
        self.last_error = error

        # Sliding surface:  s = e_dot + lambda * e
        # If the controller succeeds in keeping s = 0, then
        #     e_dot = -lambda * e  =>  e(t) = e(0) * exp(-lambda * t)
        s = error_dot + self.lambda_gain * error

        # Control law: v = lambda * e_dot + K * tanh(s / phi)
        # The first term is the "equivalent" control that pulls the state
        # along the surface. The second term is the robust "switching" term
        # that rejects disturbances.
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
        # INDEPENDENT SCALAR SMC TUNING
        # ---------------------------------------------------------
        # Each of the 6 controllers below is a STAND-ALONE scalar SMC.
        # Six of them work together, but they do not share any state.
        # This is the simplest and most pedagogical configuration.
        # =====================================================
        # Position controllers (Outer Loop) - one per translational axis.
        self.smc_x = SMC(lambda_gain=1.5, k_gain=1.0, phi_boundary=0.5, output_limit=3.0)
        self.smc_y = SMC(lambda_gain=1.5, k_gain=1.0, phi_boundary=0.5, output_limit=3.0)
        self.smc_z = SMC(lambda_gain=2.5, k_gain=2.0, phi_boundary=0.5, output_limit=8.0)

        # Attitude controllers (Inner Loop) - one per rotational axis.
        # K gains are increased compared to the position loop so that
        # the controller can overpower the unmodeled gyroscopic forces.
        self.smc_roll  = SMC(lambda_gain=6.0, k_gain=8.0, phi_boundary=0.5, output_limit=20.0)
        self.smc_pitch = SMC(lambda_gain=6.0, k_gain=8.0, phi_boundary=0.5, output_limit=20.0)
        self.smc_yaw   = SMC(lambda_gain=3.0, k_gain=4.0, phi_boundary=0.5, output_limit=10.0)

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
        # 1. OUTER LOOP - Z (Altitude)
        # ---------------------------------------------------------
        # The altitude SMC gives a virtual vertical acceleration v_z.
        # We convert v_z into a total thrust command U1 using the
        # standard quadcopter relation
        #     U1 = (m / (cos(phi) * cos(theta))) * (v_z + g)
        # The denominator is lower-bounded to avoid division by zero
        # when the drone is heavily tilted.
        # =====================================================
        v_z = self.smc_z.compute(err_z, dt)

        cos_roll_pitch = max(np.cos(self.roll) * np.cos(self.pitch), 0.1)
        U1 = (self.mass / cos_roll_pitch) * (v_z + self.gravity)
        U1 = clamp(U1, self.mass * self.gravity * 0.5, self.mass * self.gravity * 2.0)

        # =====================================================
        # 2. OUTER LOOP - X, Y (Horizontal position)
        # ---------------------------------------------------------
        # Horizontal position is controlled by TILTING the drone.
        # The position SMC gives v_x and v_y, which we convert to
        # desired roll and desired pitch. Tilts are clamped to
        # ±0.3 rad (~17 deg) to keep the drone inside the linear
        # small-angle approximation.
        # =====================================================
        v_x = clamp(self.smc_x.compute(err_x, dt), -2.5, 2.5)
        v_y = clamp(self.smc_y.compute(err_y, dt), -2.5, 2.5)

        sin_yaw = np.sin(self.yaw)
        cos_yaw = np.cos(self.yaw)

        term_phi = (self.mass / U1) * (v_x * sin_yaw - v_y * cos_yaw)
        desired_roll = np.arcsin(clamp(term_phi, -0.3, 0.3))

        term_theta = (self.mass / (U1 * np.cos(desired_roll))) * (v_x * cos_yaw + v_y * sin_yaw)
        desired_pitch = np.arcsin(clamp(term_theta, -0.3, 0.3))

        # =====================================================
        # 3. INNER LOOP - Attitude (Roll, Pitch, Yaw)
        # ---------------------------------------------------------
        # The attitude SMC converts the angle errors directly into
        # body torques. NO feedback-linearization terms are added -
        # we trust the SMC switching gain K to absorb the gyroscopic
        # cross-coupling (p*q, q*r, p*r) as bounded disturbances.
        # This is what the lecture calls the "Pure SMC" trick.
        # =====================================================
        err_roll  = (desired_roll  - self.roll  + np.pi) % (2 * np.pi) - np.pi
        err_pitch = (desired_pitch - self.pitch + np.pi) % (2 * np.pi) - np.pi
        err_yaw   = (0.0 - self.yaw + np.pi) % (2 * np.pi) - np.pi

        v_phi   = self.smc_roll.compute(err_roll, dt)
        v_theta = self.smc_pitch.compute(err_pitch, dt)
        v_psi   = self.smc_yaw.compute(err_yaw, dt)

        # Pure SMC: Treating nonlinear cross-coupling as bounded disturbances.
        # The SMC robust switching gain handles the unmodeled dynamics.
        U2 = self.Ixx * v_phi
        U3 = self.Iyy * v_theta
        U4 = self.Izz * v_psi

        # =====================================================
        # 4. CONTROL ALLOCATION (Motor Mixing)
        # ---------------------------------------------------------
        # The X-configuration mixer distributes the four control
        # "efforts" (U1..U4) to the four individual motor speeds.
        # The result is then clamped to the feasible RPM range
        # [200, 1200] before being published to Gazebo.
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