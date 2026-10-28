"""Lap monitor for the Gazebo sim: lap times, and a failure report when the car
goes off.

Uses the car's true pose (/ground_truth/odom from Gazebo) and the same rules as
tools/fast_sim.py. When the car fails it writes a folder to ~/nxp_cup_runs/:

  failure.mp4   last few seconds of /line_follower/debug, with commanded vs
                actual steering angle printed on each frame
  log.csv       last 10 s: pose, distance from centre line, speed and steering
                (commanded and actual), at 50 Hz
  summary.txt   where and why it failed

Reset the world in Gazebo to start a new attempt; the monitor re-arms by itself.
"""

import collections
import csv
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from .track_check import CarShape, TrackProgress


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class LapMonitor(Node):
    def __init__(self):
        super().__init__("lap_monitor")
        gz_share = Path(get_package_share_directory("nxp_cup_gazebo"))
        self.wheel_base = self.declare_parameter("wheel_base", 0.175).value
        sep = self.declare_parameter("wheel_separation", 0.150).value
        self.clip_s = self.declare_parameter("clip_seconds", 4.0).value
        out = self.declare_parameter("out_dir", "~/nxp_cup_runs").value
        self.out_dir = Path(os.path.expanduser(out))

        info = yaml.safe_load((gz_share / "config" / "track_info.yaml").read_text())
        cl = np.loadtxt(gz_share / "models" / "nxp_cup_track" / "centerline.csv",
                        delimiter=",", skiprows=1)
        self.track = TrackProgress(cl, info["length_m"], CarShape(self.wheel_base, sep))
        self.layout = info["layout"]

        self.cmd = (0.0, 0.0)            # commanded speed, steering angle
        self.steer_actual = float("nan")
        self.log = collections.deque(maxlen=500)                 # 10 s at 50 Hz
        self.frames = collections.deque(maxlen=int(self.clip_s * 30))
        self.last_pose = None
        self.failed = False

        self.create_subscription(Odometry, "/ground_truth/odom", self.on_pose, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Image, "/line_follower/debug", self.on_debug, qos_profile_sensor_data)
        self.get_logger().info(f"lap monitor up: layout {self.layout}, "
                               f"{info['length_m']:.2f} m lap, reports go to {self.out_dir}")

    def on_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        steer = math.atan(self.wheel_base * w / v) if abs(v) > 0.05 else 0.0
        self.cmd = (v, steer)

    def on_joints(self, msg):
        angles = [p for n, p in zip(msg.name, msg.position) if n.endswith("_steer_joint")]
        if angles:
            self.steer_actual = sum(angles) / len(angles)

    def on_debug(self, msg):
        if msg.encoding != "bgr8":
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        self.frames.append((_stamp(msg), img.reshape(msg.height, msg.width, 3).copy(),
                            self.cmd[1], self.steer_actual))

    def on_pose(self, msg):
        t = _stamp(msg)
        p = msg.pose.pose
        x, y, yaw = p.position.x, p.position.y, _yaw(p.orientation)

        # world reset in Gazebo: pose jumps or time goes backwards -> new attempt
        if self.last_pose is not None:
            lx, ly, lt = self.last_pose
            if t < lt - 0.5 or math.hypot(x - lx, y - ly) > 0.5:
                self.get_logger().info("world reset: new attempt")
                self.track.reset()
                self.failed = False
                self.log.clear()
                self.frames.clear()
        self.last_pose = (x, y, t)
        if self.failed:
            return

        event, lateral = self.track.update(x, y, yaw, t)
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.log.append((round(t, 3), round(x, 4), round(y, 4), round(yaw, 4), round(lateral, 4),
                         round(self.track.distance, 3), round(speed, 3), round(self.cmd[0], 3),
                         round(self.cmd[1], 4), round(self.steer_actual, 4)))
        if event is None:
            return
        if event[0] == "lap":
            self.get_logger().info(f"LAP {self.track.laps}: {event[1]:.2f} s, "
                                   f"max {event[2] * 100:.1f} cm off the centre line")
        else:
            self.failed = True
            self.report(event[1], t, x, y, yaw)

    def report(self, reason, t, x, y, yaw):
        run = self.out_dir / time.strftime("%Y%m%d-%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        summary = (f"layout: {self.layout}\nfailure: {reason}\nsim time: {t:.2f} s\n"
                   f"distance driven: {self.track.distance:.2f} m of {self.track.length:.2f} m\n"
                   f"pose: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f} deg\n")
        (run / "summary.txt").write_text(summary)
        with open(run / "log.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "yaw", "lateral_m", "distance_m", "speed",
                        "cmd_speed", "cmd_steer", "actual_steer"])
            w.writerows(self.log)
        if self.frames:
            self.write_clip(run / "failure.mp4")
        else:
            self.get_logger().warn("no debug images received; is the line follower publishing?")
        self.get_logger().error(f"FAILED: {reason} after {self.track.distance:.2f} m. "
                                f"Report: {run}")

    def write_clip(self, path):
        import cv2
        h, w = self.frames[0][1].shape[:2]
        scale = 2
        vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (w * scale, h * scale))
        for t, img, cmd, act in self.frames:
            big = cv2.resize(img, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
            txt = f"t={t:.2f} steer cmd {math.degrees(cmd):+5.1f} actual {math.degrees(act):+5.1f} deg"
            cv2.putText(big, txt, (6, h * scale - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            vw.write(big)
        vw.release()


def main():
    rclpy.init()
    node = LapMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
