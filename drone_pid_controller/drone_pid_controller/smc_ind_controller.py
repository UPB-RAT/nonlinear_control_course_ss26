import rclpy
from rclpy.node import Node
import numpy as np
import threading
import collections

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult

from drone_pid_controller.trajectory import get_target
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------------------
# Optional matplotlib import for the LIVE VISUALIZATION window.
# If matplotlib is not installed (e.g. on a headless server), the script
# still runs as a normal ROS 2 controller - the visualization is simply
# disabled and a warning is printed once at startup.
# ---------------------------------------------------------------------------
try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except Exception as _e:
    MATPLOTLIB_AVAILABLE = False
    _MATPLOTLIB_IMPORT_ERROR = _e

def quaternion_to_euler_scipy(w, x, y, z):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    Returned in radians for control math.
    """
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=False)

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)


class SMCDataLogger:
    """
    Thread-safe time-series data logger used by the live visualization.

    Each controller (smc_x, smc_z, smc_roll, ...) is *registered* once
    with a name and its tuning parameters. Every controller call
    appends a (time, e, e_dot, s, v) sample to its own deques.

    The visualization thread calls `snapshot()` to take a consistent
    copy of the data without holding the lock for too long.
    """

    def __init__(self, maxlen=500):
        self._lock = threading.Lock()
        self._maxlen = maxlen
        self._t0 = None
        self._times = collections.deque(maxlen=maxlen)
        # name -> dict of deques + tuning parameters
        self._controllers = {}
        # Drone state (actual + desired) for the trajectory-tracking plots
        self._state = {
            't':  collections.deque(maxlen=maxlen),
            'x':  collections.deque(maxlen=maxlen),
            'y':  collections.deque(maxlen=maxlen),
            'z':  collections.deque(maxlen=maxlen),
            'xd': collections.deque(maxlen=maxlen),
            'yd': collections.deque(maxlen=maxlen),
            'zd': collections.deque(maxlen=maxlen),
        }

    def register(self, name, lambda_gain, k_gain, phi_boundary):
        with self._lock:
            self._controllers[name] = {
                'e':      collections.deque(maxlen=self._maxlen),
                'e_dot':  collections.deque(maxlen=self._maxlen),
                's':      collections.deque(maxlen=self._maxlen),
                'v':      collections.deque(maxlen=self._maxlen),
                'lambda': lambda_gain,
                'K':      k_gain,
                'phi':    phi_boundary,
            }

    def log(self, name, t, error, error_dot, s, v):
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            self._times.append(t - self._t0)
            c = self._controllers.get(name)
            if c is None:
                return
            c['e'].append(error)
            c['e_dot'].append(error_dot)
            c['s'].append(s)
            c['v'].append(v)

    def log_state(self, t, x, y, z, xd, yd, zd):
        """Record actual (x, y, z) and desired (xd, yd, zd) drone position."""
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            self._state['t'].append(t - self._t0)
            self._state['x'].append(x)
            self._state['y'].append(y)
            self._state['z'].append(z)
            self._state['xd'].append(xd)
            self._state['yd'].append(yd)
            self._state['zd'].append(zd)

    def snapshot(self):
        """Return a deep-ish copy of the current data, safe to use without the lock."""
        with self._lock:
            return {
                'times': list(self._times),
                'controllers': {
                    name: {k: (list(v) if isinstance(v, collections.deque) else v)
                           for k, v in data.items()}
                    for name, data in self._controllers.items()
                },
                'state': {k: list(v) for k, v in self._state.items()},
            }


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
    name         : str
        Human-readable identifier used by the live visualization.
    logger       : SMCDataLogger or None
        If provided, the controller publishes its internal state
        (e, e_dot, s, v) to the logger on every call to `log()`.
    """
    def __init__(self, lambda_gain, k_gain, phi_boundary=0.1, output_limit=None,
                 name='', logger=None):
        self.lambda_gain = lambda_gain   # Slope of sliding surface
        self.k_gain = k_gain             # Robust switching gain
        self.phi_boundary = phi_boundary # Boundary layer thickness
        self.output_limit = output_limit
        self.last_error = 0.0

        # Diagnostic attributes, populated by compute() and consumed by log().
        self.last_error_dot = 0.0
        self.last_s = 0.0
        self.last_v = 0.0

        # Optional data logger for the live visualization window.
        self.name = name
        self.logger = logger
        if logger is not None:
            logger.register(name, lambda_gain, k_gain, phi_boundary)

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
        self.last_error_dot = error_dot

        # Sliding surface:  s = e_dot + lambda * e
        # If the controller succeeds in keeping s = 0, then
        #     e_dot = -lambda * e  =>  e(t) = e(0) * exp(-lambda * t)
        s = error_dot + self.lambda_gain * error
        self.last_s = s

        # Control law: v = lambda * e_dot + K * tanh(s / phi)
        # The first term is the "equivalent" control that pulls the state
        # along the surface. The second term is the robust "switching" term
        # that rejects disturbances.
        v = self.lambda_gain * error_dot + self.k_gain * np.tanh(s / self.phi_boundary)

        if self.output_limit is not None:
            v = clamp(v, -self.output_limit, self.output_limit)
        self.last_v = v

        return v

    def log(self, t, error):
        """
        Push the most recent (e, e_dot, s, v) to the data logger.

        Call this from the controller node right after `compute()`.
        Does nothing if no logger was attached at construction time.
        """
        if self.logger is not None:
            self.logger.log(self.name, t, error,
                            self.last_error_dot, self.last_s, self.last_v)

class QuadcopterSMC(Node):

    def __init__(self, logger=None):
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

        # Store the data logger as an instance attribute so that
        # control_step() can push actual/desired position samples to
        # the live visualization dashboard. We MUST keep the name
        # "logger" because rclpy.Node already uses "_logger" internally.
        self.logger = logger

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
        # If a logger is passed in, every controller publishes its
        # (e, e_dot, s, v) samples for the live visualization window.
        # =====================================================
        # Position controllers (Outer Loop) - one per translational axis.
        self.smc_x = SMC(lambda_gain=1.5, k_gain=1.0, phi_boundary=0.5, output_limit=3.0,
                         name='x', logger=logger)
        self.smc_y = SMC(lambda_gain=1.5, k_gain=1.0, phi_boundary=0.5, output_limit=3.0,
                         name='y', logger=logger)
        self.smc_z = SMC(lambda_gain=2.5, k_gain=2.0, phi_boundary=0.5, output_limit=8.0,
                         name='z', logger=logger)

        # Attitude controllers (Inner Loop) - one per rotational axis.
        # K gains are increased compared to the position loop so that
        # the controller can overpower the unmodeled gyroscopic forces.
        self.smc_roll  = SMC(lambda_gain=6.0, k_gain=8.0, phi_boundary=0.5, output_limit=20.0,
                             name='roll',  logger=logger)
        self.smc_pitch = SMC(lambda_gain=6.0, k_gain=8.0, phi_boundary=0.5, output_limit=20.0,
                             name='pitch', logger=logger)
        self.smc_yaw   = SMC(lambda_gain=3.0, k_gain=4.0, phi_boundary=0.5, output_limit=10.0,
                             name='yaw',   logger=logger)

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

        # Push (actual, desired) position to the live visualization logger.
        if self.logger is not None:
            self.logger.log_state(
                t,
                self.current_x, self.current_y, self.current_z,
                tx, ty, tz,
            )

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
        self.smc_z.log(t, err_z)

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
        self.smc_x.log(t, err_x)
        self.smc_y.log(t, err_y)

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
        self.smc_roll.log(t,  err_roll)
        self.smc_pitch.log(t, err_pitch)
        self.smc_yaw.log(t,   err_yaw)

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

def run_visualization(logger):
    """
    Run the live SMC visualization in a SEPARATE THREAD.

    Six subplots are updated every 200 ms from the data produced by
    the ROS 2 control loop:

        Row 1
        -----
        1. Phase portrait (e vs. e_dot) of the z-axis with the
           sliding line  e_dot = -lambda * e.
        2. Top-down 2-D trajectory (x-y) - desired path vs. actual
           path; the gap between them is the tracking error.
        3. Altitude tracking  z(t) - desired vs. current.

        Row 2
        -----
        4. Sliding variable  s(t)  for all 6 axes.
        5. Tracking error    e(t)  for x, y, z.
        6. Control input     v(t)  for all 6 axes (chattering view).
    """
    # Pick a modern style if available; silently fall back to the default.
    for _style in ('seaborn-v0_8-whitegrid', 'seaborn-whitegrid', 'ggplot'):
        try:
            plt.style.use(_style)
            break
        except Exception:
            continue

    # matplotlib-level typography defaults
    plt.rcParams.update({
        'font.size':        10,
        'axes.titlesize':   12,
        'axes.labelsize':   10,
        'legend.fontsize':  9,
        'lines.linewidth':  1.6,
        'grid.alpha':       0.4,
    })

    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    fig.suptitle('Sliding Mode Controller  -  Live Tracking Dashboard',
                 fontsize=15, fontweight='bold', y=0.995)

    # Color palette - one vivid color per controlled axis
    AXIS_COLORS = {
        'x':     '#1f77b4',   # blue
        'y':     '#ff7f0e',   # orange
        'z':     '#2ca02c',   # green
        'roll':  '#d62728',   # red
        'pitch': '#9467bd',   # purple
        'yaw':   '#8c564b',   # brown
    }
    COLOR_DESIRED = '#444444'   # dark gray for the planned path
    COLOR_CURRENT = '#2ca02c'   # green for the actual path

    # -----------------------------------------------------------------
    # Plot 1: Phase portrait (z-axis) with sliding line
    # -----------------------------------------------------------------
    ax1 = axes[0, 0]
    ax1.set_xlabel('Error  $e_z$  (m)')
    ax1.set_ylabel('Error rate  $\\dot{e}_z$  (m/s)')
    ax1.set_title('Phase Portrait  (z-axis)')
    ax1.axhline(0, color='k', linewidth=0.5)
    ax1.axvline(0, color='k', linewidth=0.5)
    line_traj,    = ax1.plot([], [], color=COLOR_CURRENT, linewidth=1.8,
                             label='Trajectory')
    line_sliding, = ax1.plot([], [], color='#d62728', linestyle='--',
                             linewidth=1.6,
                             label='Sliding line  $\\dot e = -\\lambda e$')
    ax1.legend(loc='best')

    # -----------------------------------------------------------------
    # Plot 2: Top-down 2-D trajectory (x-y)
    # -----------------------------------------------------------------
    ax2 = axes[0, 1]
    ax2.set_xlabel('x  (m)')
    ax2.set_ylabel('y  (m)')
    ax2.set_title('Top-Down Trajectory  (desired vs. actual)')
    ax2.set_aspect('equal', adjustable='datalim')
    line_xy_desired, = ax2.plot([], [], color=COLOR_DESIRED, linestyle='--',
                                linewidth=1.4, label='Desired', alpha=0.8)
    line_xy_current, = ax2.plot([], [], color=COLOR_CURRENT, linewidth=2.0,
                                label='Current')
    drone_marker,    = ax2.plot([], [], marker='o', markersize=8,
                                color=COLOR_CURRENT, markeredgecolor='black',
                                markeredgewidth=1.0, linestyle='None',
                                label='Drone')
    ax2.legend(loc='best')

    # -----------------------------------------------------------------
    # Plot 3: Altitude tracking  z(t)
    # -----------------------------------------------------------------
    ax3 = axes[0, 2]
    ax3.set_xlabel('Time  (s)')
    ax3.set_ylabel('Altitude  $z$  (m)')
    ax3.set_title('Altitude Tracking  (desired vs. actual)')
    line_z_desired, = ax3.plot([], [], color=COLOR_DESIRED, linestyle='--',
                               linewidth=1.4, label='Desired', alpha=0.8)
    line_z_current, = ax3.plot([], [], color=COLOR_CURRENT, linewidth=2.0,
                               label='Current')
    ax3.legend(loc='best')

    # -----------------------------------------------------------------
    # Plot 4: Sliding variable  s(t)  for every axis
    # -----------------------------------------------------------------
    ax4 = axes[1, 0]
    ax4.set_xlabel('Time  (s)')
    ax4.set_ylabel('Sliding variable  $s$')
    ax4.set_title('Sliding Variable  $s(t)$')
    ax4.axhline(0, color='k', linewidth=0.5, linestyle='--')
    s_lines = {}

    # -----------------------------------------------------------------
    # Plot 5: Tracking error  e(t)  for x, y, z
    # -----------------------------------------------------------------
    ax5 = axes[1, 1]
    ax5.set_xlabel('Time  (s)')
    ax5.set_ylabel('Tracking error  $e$')
    ax5.set_title('Tracking Error  $e(t)$  (x, y, z)')
    ax5.axhline(0, color='k', linewidth=0.5, linestyle='--')
    e_lines = {}

    # -----------------------------------------------------------------
    # Plot 6: Control input  v(t)  for every axis
    # -----------------------------------------------------------------
    ax6 = axes[1, 2]
    ax6.set_xlabel('Time  (s)')
    ax6.set_ylabel('Control input  $v$')
    ax6.set_title('Control Input  $v(t)$  (chattering)')
    v_lines = {}

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------
    def _rescale(ax, values, pad=0.1):
        if not values:
            return
        v_min, v_max = min(values), max(values)
        margin = pad * (v_max - v_min + 1e-6)
        ax.set_ylim(v_min - margin, v_max + margin)

    def _ensure_line(ax, registry, name, color):
        if name not in registry:
            line, = ax.plot([], [], color=color, label=name, linewidth=1.4)
            registry[name] = line
        return registry[name]

    # -----------------------------------------------------------------
    # Animation update
    # -----------------------------------------------------------------
    def update(frame):
        snap = logger.snapshot()
        times = snap['times']
        state = snap['state']
        controllers = snap['controllers']

        # ---- Plot 1: phase portrait (z) ---------------------------
        if 'z' in controllers:
            cz = controllers['z']
            if len(cz['e']) > 0:
                line_traj.set_data(cz['e'], cz['e_dot'])
                e_arr = np.array(cz['e'])
                e_min, e_max = float(e_arr.min()), float(e_arr.max())
                margin = 0.15 * (e_max - e_min + 1e-6) + 0.1
                ax1.set_xlim(e_min - margin, e_max + margin)
                e_range = np.array([e_min - margin, e_max + margin])
                lam = cz.get('lambda', 1.5)
                line_sliding.set_data(e_range, -lam * e_range)
                _rescale(ax1, cz['e_dot'])

        # ---- Plots 2 & 3: trajectory (x-y) and altitude (z) ------
        st = state['t']
        if len(st) >= 1:
            t_now = st[-1]
            line_xy_desired.set_data(state['xd'], state['yd'])
            line_xy_current.set_data(state['x'],  state['y'])
            drone_marker.set_data([state['x'][-1]], [state['y'][-1]])

            line_z_desired.set_data(st, state['zd'])
            line_z_current.set_data(st, state['z'])

            # auto-rescale trajectory plot with margin
            all_x = state['x'] + state['xd']
            all_y = state['y'] + state['yd']
            if all_x and all_y:
                x_min, x_max = min(all_x), max(all_x)
                y_min, y_max = min(all_y), max(all_y)
                m = 0.15 * max(x_max - x_min, y_max - y_min, 1e-3) + 0.1
                ax2.set_xlim(x_min - m, x_max + m)
                ax2.set_ylim(y_min - m, y_max + m)

            # rescale altitude plot
            all_z = state['z'] + state['zd']
            if all_z:
                ax3.set_xlim(st[0], st[-1])
                _rescale(ax3, all_z)

        # ---- Plot 4: sliding variable s(t) -----------------------
        all_s = []
        for name, cdata in controllers.items():
            color = AXIS_COLORS.get(name, 'k')
            line = _ensure_line(ax4, s_lines, name, color)
            n = len(cdata['s'])
            if n > 0:
                t_rel = times[-n:]
                line.set_data(t_rel, cdata['s'])
                all_s.extend(cdata['s'])
        if times:
            ax4.set_xlim(times[0], times[-1])
            _rescale(ax4, all_s)
        if ax4.get_legend() is None:
            ax4.legend(loc='best', ncol=3)

        # ---- Plot 5: tracking error e(t) for x, y, z -------------
        for axis in ('x', 'y', 'z'):
            if axis in controllers:
                color = AXIS_COLORS[axis]
                line = _ensure_line(ax5, e_lines, axis, color)
                cd = controllers[axis]
                n = len(cd['e'])
                if n > 0:
                    t_rel = times[-n:]
                    line.set_data(t_rel, cd['e'])
        if times:
            ax5.set_xlim(times[0], times[-1])
            all_e = [v for c in controllers.values() for v in c['e']]
            _rescale(ax5, all_e)
        if ax5.get_legend() is None:
            ax5.legend(loc='best')

        # ---- Plot 6: control input v(t) --------------------------
        all_v = []
        for name, cdata in controllers.items():
            color = AXIS_COLORS.get(name, 'k')
            line = _ensure_line(ax6, v_lines, name, color)
            n = len(cdata['v'])
            if n > 0:
                t_rel = times[-n:]
                line.set_data(t_rel, cdata['v'])
                all_v.extend(cdata['v'])
        if times:
            ax6.set_xlim(times[0], times[-1])
            _rescale(ax6, all_v)
        if ax6.get_legend() is None:
            ax6.legend(loc='best', ncol=3)

        return []

    anim = FuncAnimation(fig, update, interval=200, blit=False,
                         cache_frame_data=False)
    plt.show()


def _ros_loop(logger):
    """
    Run the ROS 2 control loop in a sub-thread.

    Why a sub-thread?  TkAgg (and every other GUI backend) refuses to
    start a GUI outside the MAIN thread, so matplotlib's plt.show()
    MUST be called from the main thread.  We therefore invert the
    usual layout: ROS spins in this worker, matplotlib runs in main.
    """
    import rclpy.executors
    node = QuadcopterSMC(logger=logger)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.destroy_node()


def main():
    # ----- Live visualization setup (optional but recommended) -----
    # The matplotlib GUI MUST run in the main thread. ROS 2 is moved
    # to a background worker so plt.show() can own the main thread.
    if not MATPLOTLIB_AVAILABLE:
        # Fall back to running the controller without the GUI.
        print(
            "[smc_ind_controller] matplotlib is not available "
            "(import error: {!r}). Running without live visualization."
            .format(_MATPLOTLIB_IMPORT_ERROR)
        )
        rclpy.init()
        node = QuadcopterSMC(logger=None)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    # Visualization path: matplotlib (main thread) + ROS (sub-thread).
    rclpy.init()
    logger = SMCDataLogger(maxlen=500)

    ros_thread = threading.Thread(
        target=_ros_loop, args=(logger,), daemon=True
    )
    ros_thread.start()

    try:
        # Blocks the main thread until the user closes the dashboard.
        run_visualization(logger)
    finally:
        # Window closed (or Ctrl+C): shut ROS down and join the worker.
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)

if __name__ == '__main__':
    main()