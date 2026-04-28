#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from utils.math_utils import normalize_angle
from utils.pid import PID
from utils.trajectory import Figure8Trajectory, SquareTrajectory
from utils.dashboard import LivePIDDashboard


class Controller(Node):

    def __init__(self):
        super().__init__("controller")

        self.pose = None
        self.dt = 0.05

        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.cb, 10)

        # trajectory (GLOBAL, no segmentation)
        # self.traj = Figure8Trajectory(a=3.0)
        self.traj = SquareTrajectory(size=5)

        # PID
        self.dist_pid = PID(1.4, 0.0, 0.15, self.dt, limit=2.0)
        self.head_pid = PID(6.0, 0.0, 0.35, self.dt, limit=4.0)

        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info("Controller Started")

        self.history = []

        self.dashboard = LivePIDDashboard(
            enabled=True,
            update_period=0.2,
            initial_gains={
                "distance_kp": self.dist_pid.kp,
                "distance_ki": self.dist_pid.ki,
                "distance_kd": self.dist_pid.kd,
                "heading_kp": self.head_pid.kp,
                "heading_ki": self.head_pid.ki,
                "heading_kd": self.head_pid.kd,
            },
            on_gain_change=self._update_gains,
            on_reset_pid_memory=self._reset_pids,
        )




    def _update_gains(self, gains):
        self.dist_pid.set_gains(
            gains["distance_kp"],
            gains["distance_ki"],
            gains["distance_kd"],
        )
        self.head_pid.set_gains(
            gains["heading_kp"],
            gains["heading_ki"],
            gains["heading_kd"],
        )


    def _reset_pids(self):
        self.dist_pid.reset_memory()
        self.head_pid.reset_memory()


    def cb(self, msg):
        self.pose = msg

    def loop(self):

        if self.pose is None:
            return

        x_d, y_d, xd, yd = self.traj.step(self.dt)

        dx = x_d - self.pose.x
        dy = y_d - self.pose.y

        dist = math.sqrt(dx*dx + dy*dy)

        desired_theta = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_theta - self.pose.theta)

        cmd = Twist()

        # simple switching controller (same as Claude logic)
        if abs(heading_error) < 0.35:
            cmd.linear.x = self.dist_pid.compute(dist)
        else:
            cmd.linear.x = 0.0

        cmd.angular.z = self.head_pid.compute(heading_error)

        cmd.linear.x = max(0.0, min(2.0, cmd.linear.x))
        cmd.angular.z = max(-4.0, min(4.0, cmd.angular.z))

        self.cmd_pub.publish(cmd)

        self.history.append({
            "time": self.get_clock().now().nanoseconds * 1e-9,
            "desired_x": x_d,
            "desired_y": y_d,
            "actual_x": self.pose.x,
            "actual_y": self.pose.y,
            "error_x": dx,
            "error_y": dy,
            "distance_error": dist,
            "heading_error": heading_error,
            "linear_cmd": cmd.linear.x,
            "angular_cmd": cmd.angular.z,
            "distance_p": self.dist_pid.p_term,
            "distance_i": self.dist_pid.i_term,
            "distance_d": self.dist_pid.d_term,
            "distance_output": self.dist_pid.output,
            "heading_p": self.head_pid.p_term,
            "heading_i": self.head_pid.i_term,
            "heading_d": self.head_pid.d_term,
            "heading_output": self.head_pid.output,
        })

        self.dashboard.update(self.history)


def main():
    rclpy.init()
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()