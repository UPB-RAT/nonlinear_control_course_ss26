# drone_smc_controller/smc_controller.py
from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import csv
import math
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from actuator_msgs.msg import Actuators
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R

from drone_smc_controller.min_snap_trajectory import MinimumSnapTrajectory

# Optional dashboard import
try:
    from drone_smc_controller.live_smc_dashboard import LiveSMCDashboard
except Exception:
    LiveSMCDashboard = None


# =========================
# Utilities
# =========================

def clamp(v, low, high):
    return max(low, min(v, high))


def smooth_sign(s, eps=0.1):
    return np.tanh(s / max(eps, 1e-6))


def wrap_angle_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def quaternion_to_euler_deg(w, x, y, z):
    r = R.from_quat([x, y, z, w])
    return r.as_euler("xyz", degrees=True)


# =========================
# Config dataclasses
# =========================

@dataclass
class TrajectoryCfg:
    shape: str
    A: float
    radius: float
    height: float
    climb_rate: float
    side: float
    segment_time: float
    num_points: int


@dataclass
class GainsCfg:
    lambda_x: float
    lambda_y: float
    lambda_z: float
    k_x: float
    k_y: float
    k_z: float
    lambda_roll: float
    lambda_pitch: float
    lambda_yaw: float
    k_roll: float
    k_pitch: float
    k_yaw: float


@dataclass
class EpsCfg:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class LimitsCfg:
    max_roll_deg: float
    max_pitch_deg: float
    max_body_rate_dps: float
    max_u_roll: float
    max_u_pitch: float
    max_u_z: float
    max_u_yaw: float


@dataclass
class MotorsCfg:
    base_speed: float
    min_speed: float
    max_speed: float


@dataclass
class RuntimeCfg:
    control_dt: float
    vel_filter_alpha: float
    max_path_len: int
    csv_path: str
    fail_safe_error: float
    yaw_ref_deg: float


@dataclass
class DashboardCfg:
    enabled: bool
    update_period: float
    history_len: int


# =========================
# Controller Node
# =========================

class QuadcopterSMC(Node):
    def __init__(self):
        super().__init__("quadcopter_smc")
        self._declare_parameters()
        self._load_config()

        self._setup_ros_interfaces()
        self._init_state()
        self._init_trajectory()
        self._init_logging()
        self._init_dashboard()

        self.create_timer(self.runtime.control_dt, self.control_loop)
        self.create_timer(self.dashboard_cfg.update_period, self.dashboard_timer_callback)

    # ---------- Initialization ----------
    def _declare_parameters(self):
        # trajectory
        self.declare_parameter("trajectory.shape", "circle")
        self.declare_parameter("trajectory.A", 2.0)
        self.declare_parameter("trajectory.radius", 0.5)
        self.declare_parameter("trajectory.height", 1.0)
        self.declare_parameter("trajectory.climb_rate", 0.1)
        self.declare_parameter("trajectory.side", 4.0)
        self.declare_parameter("trajectory.segment_time", 4.0)
        self.declare_parameter("trajectory.num_points", 30)

        # gains
        self.declare_parameter("gains.lambda_x", 0.6)
        self.declare_parameter("gains.lambda_y", 0.6)
        self.declare_parameter("gains.lambda_z", 1.0)
        self.declare_parameter("gains.k_x", 4.0)
        self.declare_parameter("gains.k_y", 4.0)
        self.declare_parameter("gains.k_z", 45.0)
        self.declare_parameter("gains.lambda_roll", 1.5)
        self.declare_parameter("gains.lambda_pitch", 1.5)
        self.declare_parameter("gains.lambda_yaw", 1.0)
        self.declare_parameter("gains.k_roll", 8.0)
        self.declare_parameter("gains.k_pitch", 8.0)
        self.declare_parameter("gains.k_yaw", 0.0)

        # eps
        self.declare_parameter("eps.x", 0.30)
        self.declare_parameter("eps.y", 0.30)
        self.declare_parameter("eps.z", 0.30)
        self.declare_parameter("eps.roll", 0.30)
        self.declare_parameter("eps.pitch", 0.30)
        self.declare_parameter("eps.yaw", 0.30)

        # limits
        self.declare_parameter("limits.max_roll_deg", 7.0)
        self.declare_parameter("limits.max_pitch_deg", 7.0)
        self.declare_parameter("limits.max_body_rate_dps", 120.0)
        self.declare_parameter("limits.max_u_roll", 20.0)
        self.declare_parameter("limits.max_u_pitch", 20.0)
        self.declare_parameter("limits.max_u_z", 120.0)
        self.declare_parameter("limits.max_u_yaw", 30.0)

        # motors
        self.declare_parameter("motors.base_speed", 656.0)
        self.declare_parameter("motors.min", 400.0)
        self.declare_parameter("motors.max", 800.0)

        # runtime
        self.declare_parameter("yaw_ref_deg", 0.0)
        self.declare_parameter("runtime.control_dt", 0.02)
        self.declare_parameter("runtime.vel_filter_alpha", 0.20)
        self.declare_parameter("runtime.max_path_len", 1000)
        self.declare_parameter("runtime.csv_path", "tracking_error.csv")
        self.declare_parameter("runtime.fail_safe_error", 8.0)

        # dashboard
        self.declare_parameter("dashboard.enabled", True)
        self.declare_parameter("dashboard.update_period", 0.1)
        self.declare_parameter("dashboard.history_len", 400)

    def _load_config(self):
        self.traj_cfg = TrajectoryCfg(
            shape=self.get_parameter("trajectory.shape").value,
            A=float(self.get_parameter("trajectory.A").value),
            radius=float(self.get_parameter("trajectory.radius").value),
            height=float(self.get_parameter("trajectory.height").value),
            climb_rate=float(self.get_parameter("trajectory.climb_rate").value),
            side=float(self.get_parameter("trajectory.side").value),
            segment_time=float(self.get_parameter("trajectory.segment_time").value),
            num_points=int(self.get_parameter("trajectory.num_points").value),
        )
        self.gains = GainsCfg(
            lambda_x=float(self.get_parameter("gains.lambda_x").value),
            lambda_y=float(self.get_parameter("gains.lambda_y").value),
            lambda_z=float(self.get_parameter("gains.lambda_z").value),
            k_x=float(self.get_parameter("gains.k_x").value),
            k_y=float(self.get_parameter("gains.k_y").value),
            k_z=float(self.get_parameter("gains.k_z").value),
            lambda_roll=float(self.get_parameter("gains.lambda_roll").value),
            lambda_pitch=float(self.get_parameter("gains.lambda_pitch").value),
            lambda_yaw=float(self.get_parameter("gains.lambda_yaw").value),
            k_roll=float(self.get_parameter("gains.k_roll").value),
            k_pitch=float(self.get_parameter("gains.k_pitch").value),
            k_yaw=float(self.get_parameter("gains.k_yaw").value),
        )
        self.eps = EpsCfg(
            x=float(self.get_parameter("eps.x").value),
            y=float(self.get_parameter("eps.y").value),
            z=float(self.get_parameter("eps.z").value),
            roll=float(self.get_parameter("eps.roll").value),
            pitch=float(self.get_parameter("eps.pitch").value),
            yaw=float(self.get_parameter("eps.yaw").value),
        )
        self.limits = LimitsCfg(
            max_roll_deg=float(self.get_parameter("limits.max_roll_deg").value),
            max_pitch_deg=float(self.get_parameter("limits.max_pitch_deg").value),
            max_body_rate_dps=float(self.get_parameter("limits.max_body_rate_dps").value),
            max_u_roll=float(self.get_parameter("limits.max_u_roll").value),
            max_u_pitch=float(self.get_parameter("limits.max_u_pitch").value),
            max_u_z=float(self.get_parameter("limits.max_u_z").value),
            max_u_yaw=float(self.get_parameter("limits.max_u_yaw").value),
        )
        self.motors = MotorsCfg(
            base_speed=float(self.get_parameter("motors.base_speed").value),
            min_speed=float(self.get_parameter("motors.min").value),
            max_speed=float(self.get_parameter("motors.max").value),
        )
        self.runtime = RuntimeCfg(
            control_dt=float(self.get_parameter("runtime.control_dt").value),
            vel_filter_alpha=float(self.get_parameter("runtime.vel_filter_alpha").value),
            max_path_len=int(self.get_parameter("runtime.max_path_len").value),
            csv_path=str(self.get_parameter("runtime.csv_path").value),
            fail_safe_error=float(self.get_parameter("runtime.fail_safe_error").value),
            yaw_ref_deg=float(self.get_parameter("yaw_ref_deg").value),
        )
        self.dashboard_cfg = DashboardCfg(
            enabled=bool(self.get_parameter("dashboard.enabled").value),
            update_period=float(self.get_parameter("dashboard.update_period").value),
            history_len=int(self.get_parameter("dashboard.history_len").value),
        )

    def _setup_ros_interfaces(self):
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.path_pub = self.create_publisher(Path, "/drone_path", 10)
        self.motor_pub = self.create_publisher(Actuators, "/X3/gazebo/command/motor_speed", 10)
        self.pose_sub = self.create_subscription(PoseArray, "/world/quadcopter/pose/info", self.pose_callback, qos)

    def _init_state(self):
        self.current_pose = None
        self.current_x = self.current_y = self.current_z = 0.0
        self.roll = self.pitch = self.yaw = 0.0

        self.vx = self.vy = self.vz = 0.0
        self.prev_x = self.prev_y = self.prev_z = None
        self.prev_roll = self.prev_pitch = self.prev_yaw = None
        self.initialized = False

        self.last_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"

    def _init_trajectory(self):
        params = {
            "A": self.traj_cfg.A,
            "r": self.traj_cfg.radius,
            "h": self.traj_cfg.height,
            "climb": self.traj_cfg.climb_rate,
            "side": self.traj_cfg.side,
        }
        self.traj = MinimumSnapTrajectory(
            shape=self.traj_cfg.shape,
            shape_params=params,
            times=self.traj_cfg.segment_time,
            num_points=self.traj_cfg.num_points,
        )

    def _init_logging(self):
        self.history = deque(maxlen=self.dashboard_cfg.history_len)
        self.csv = None
        self.writer = None
        try:
            self.csv = open(self.runtime.csv_path, "w", newline="")
            self.writer = csv.writer(self.csv)
            self.writer.writerow(["time", "tx", "ty", "tz", "x", "y", "z", "error"])
        except Exception as e:
            self.get_logger().warn(f"CSV disabled: {e}")

    def _init_dashboard(self):
        if LiveSMCDashboard is None:
            self.dashboard = None
            return
        self.dashboard = LiveSMCDashboard(
            enabled=self.dashboard_cfg.enabled,
            update_period=self.dashboard_cfg.update_period,
            initial_params={
                "k_x": self.gains.k_x,
                "k_y": self.gains.k_y,
                "k_z": self.gains.k_z,
                "k_roll": self.gains.k_roll,
                "k_pitch": self.gains.k_pitch,
                "k_yaw": self.gains.k_yaw,
                "lambda_x": self.gains.lambda_x,
                "lambda_y": self.gains.lambda_y,
                "lambda_z": self.gains.lambda_z,
            },
            on_param_change=self._on_dashboard_gain_change,
            on_reset_controller_memory=self._reset_controller_memory,
        )

    # ---------- Callbacks ----------
    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return
        p = msg.poses[1]
        self.current_pose = p
        self.current_x = p.position.x
        self.current_y = p.position.y
        self.current_z = p.position.z
        self.roll, self.pitch, self.yaw = quaternion_to_euler_deg(
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
        )

        if not self.initialized:
            self.prev_x, self.prev_y, self.prev_z = self.current_x, self.current_y, self.current_z
            self.prev_roll, self.prev_pitch, self.prev_yaw = self.roll, self.pitch, self.yaw
            self.initialized = True

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "world"
        pose.pose = p
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.runtime.max_path_len:
            self.path_msg.poses.pop(0)
        self.path_pub.publish(self.path_msg)

    def dashboard_timer_callback(self):
        if not self.dashboard_cfg.enabled or self.dashboard is None:
            return
        try:
            self.dashboard.update(self.history, force=True)
        except Exception as e:
            self.get_logger().warn(f"Dashboard update failed: {e}", throttle_duration_sec=2.0)

    # ---------- Core control ----------
    def control_loop(self):
        if self.current_pose is None or not self.initialized:
            return

        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 1e-3)
        self.last_time = now
        t = (now - self.start_time).nanoseconds / 1e9

        # desired
        tx, ty, tz, _ = self.traj.get_goal(t)
        d = self.traj.get_derivatives(t)
        vdx_w, vdy_w, vdz_w = d["vel"]

        # errors
        ex_w = tx - self.current_x
        ey_w = ty - self.current_y
        ez = tz - self.current_z
        err = float(np.sqrt(ex_w**2 + ey_w**2 + ez**2))

        if self.writer is not None:
            self.writer.writerow([t, tx, ty, tz, self.current_x, self.current_y, self.current_z, err])

        if err > self.runtime.fail_safe_error:
            self._publish_motors(now, [self.motors.base_speed] * 4)
            return

        # velocity estimate
        self.vx = self.runtime.vel_filter_alpha * (self.current_x - self.prev_x) / dt + (1.0 - self.runtime.vel_filter_alpha) * self.vx
        self.vy = self.runtime.vel_filter_alpha * (self.current_y - self.prev_y) / dt + (1.0 - self.runtime.vel_filter_alpha) * self.vy
        self.vz = self.runtime.vel_filter_alpha * (self.current_z - self.prev_z) / dt + (1.0 - self.runtime.vel_filter_alpha) * self.vz
        self.prev_x, self.prev_y, self.prev_z = self.current_x, self.current_y, self.current_z

        # world->body(yaw)
        yaw_rad = math.radians(self.yaw)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)

        ex_b = cos_y * ex_w + sin_y * ey_w
        ey_b = -sin_y * ex_w + cos_y * ey_w

        vdx_b = cos_y * vdx_w + sin_y * vdy_w
        vdy_b = -sin_y * vdx_w + cos_y * vdy_w

        vx_b = cos_y * self.vx + sin_y * self.vy
        vy_b = -sin_y * self.vx + cos_y * self.vy

        # sliding surfaces (position)
        sx = (vdx_b - vx_b) + self.gains.lambda_x * ex_b
        sy = (vdy_b - vy_b) + self.gains.lambda_y * ey_b
        sz = (vdz_w - self.vz) + self.gains.lambda_z * ez

        desired_roll = clamp(self.gains.k_y * smooth_sign(sy, self.eps.y), -self.limits.max_roll_deg, self.limits.max_roll_deg)
        desired_pitch = clamp(self.gains.k_x * smooth_sign(sx, self.eps.x), -self.limits.max_pitch_deg, self.limits.max_pitch_deg)
        u_z = clamp(self.gains.k_z * smooth_sign(sz, self.eps.z), -self.limits.max_u_z, self.limits.max_u_z)

        # rates
        roll_rate = clamp((self.roll - self.prev_roll) / dt, -self.limits.max_body_rate_dps, self.limits.max_body_rate_dps)
        pitch_rate = clamp((self.pitch - self.prev_pitch) / dt, -self.limits.max_body_rate_dps, self.limits.max_body_rate_dps)
        yaw_rate = clamp((self.yaw - self.prev_yaw) / dt, -self.limits.max_body_rate_dps, self.limits.max_body_rate_dps)
        self.prev_roll, self.prev_pitch, self.prev_yaw = self.roll, self.pitch, self.yaw

        # sliding surfaces (attitude)
        roll_error = -desired_roll - self.roll
        pitch_error = desired_pitch - self.pitch
        yaw_error = wrap_angle_deg(self.runtime.yaw_ref_deg - self.yaw)

        s_roll = -roll_rate + self.gains.lambda_roll * roll_error
        s_pitch = -pitch_rate + self.gains.lambda_pitch * pitch_error
        s_yaw = -yaw_rate + self.gains.lambda_yaw * yaw_error

        u_roll = clamp(self.gains.k_roll * smooth_sign(s_roll, self.eps.roll), -self.limits.max_u_roll, self.limits.max_u_roll)
        u_pitch = clamp(self.gains.k_pitch * smooth_sign(s_pitch, self.eps.pitch), -self.limits.max_u_pitch, self.limits.max_u_pitch)
        u_yaw = clamp(self.gains.k_yaw * smooth_sign(s_yaw, self.eps.yaw), -self.limits.max_u_yaw, self.limits.max_u_yaw)

        # mixer (PID-proven)
        motors = self._mix_commands(u_roll, u_pitch, u_yaw, u_z)
        self._publish_motors(now, motors)

        self.history.append({
            "time": t, "tx": tx, "ty": ty, "tz": tz,
            "x": self.current_x, "y": self.current_y, "z": self.current_z,
            "ex": ex_w, "ey": ey_w, "ez": ez, "err": err,
            "sx": sx, "sy": sy, "sz": sz,
            "roll": self.roll, "pitch": self.pitch,
            "roll_des": desired_roll, "pitch_des": desired_pitch,
            "u_roll": u_roll, "u_pitch": u_pitch, "u_z": u_z, "u_yaw": u_yaw,
            "m0": motors[0], "m1": motors[1], "m2": motors[2], "m3": motors[3],
        })

        self.get_logger().info(f"Err={err:.2f}", throttle_duration_sec=0.5)

    # ---------- Helpers ----------
    def _mix_commands(self, u_roll, u_pitch, u_yaw, u_z):
        base_thrust = self.motors.base_speed + u_z

        d0 = -u_roll - u_pitch - u_yaw
        d1 = +u_roll + u_pitch - u_yaw
        d2 = +u_roll - u_pitch + u_yaw
        d3 = -u_roll + u_pitch + u_yaw

        headroom_up = max(0.0, self.motors.max_speed - base_thrust)
        headroom_down = max(0.0, base_thrust - self.motors.min_speed)

        max_up = max(d0, d1, d2, d3)
        max_down = -min(d0, d1, d2, d3)

        scale_up = headroom_up / max_up if max_up > 0.0 else 1.0
        scale_down = headroom_down / max_down if max_down > 0.0 else 1.0
        scale = min(scale_up, scale_down, 1.0)

        if scale < 1.0:
            u_roll *= scale
            u_pitch *= scale
            u_yaw *= scale
            d0 = -u_roll - u_pitch - u_yaw
            d1 = +u_roll + u_pitch - u_yaw
            d2 = +u_roll - u_pitch + u_yaw
            d3 = -u_roll + u_pitch + u_yaw

        m0 = clamp(base_thrust + d0, self.motors.min_speed, self.motors.max_speed)
        m1 = clamp(base_thrust + d1, self.motors.min_speed, self.motors.max_speed)
        m2 = clamp(base_thrust + d2, self.motors.min_speed, self.motors.max_speed)
        m3 = clamp(base_thrust + d3, self.motors.min_speed, self.motors.max_speed)

        return [m0, m1, m2, m3]

    def _publish_motors(self, now, motors):
        if not all(np.isfinite(m) for m in motors):
            return
        cmd = Actuators()
        cmd.header = Header()
        cmd.header.stamp = now.to_msg()
        cmd.velocity = [float(m) for m in motors]
        self.motor_pub.publish(cmd)

    def _on_dashboard_gain_change(self, p):
        self.gains.k_x = float(p["k_x"])
        self.gains.k_y = float(p["k_y"])
        self.gains.k_z = float(p["k_z"])
        self.gains.k_roll = float(p["k_roll"])
        self.gains.k_pitch = float(p["k_pitch"])
        self.gains.k_yaw = float(p["k_yaw"])
        self.gains.lambda_x = float(p["lambda_x"])
        self.gains.lambda_y = float(p["lambda_y"])
        self.gains.lambda_z = float(p["lambda_z"])

    def _reset_controller_memory(self):
        self.vx = self.vy = self.vz = 0.0
        if self.current_pose is not None:
            self.prev_x, self.prev_y, self.prev_z = self.current_x, self.current_y, self.current_z
            self.prev_roll, self.prev_pitch, self.prev_yaw = self.roll, self.pitch, self.yaw

    def destroy_node(self):
        if self.csv is not None:
            self.csv.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = QuadcopterSMC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()