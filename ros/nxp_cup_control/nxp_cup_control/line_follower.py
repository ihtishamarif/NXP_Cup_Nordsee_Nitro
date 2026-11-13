"""ROS 2 node: camera image in, /cmd_vel out, using lane_tracker.LineFollower.

Subscribes  /camera/image_raw  (sensor_msgs/Image, rgb8 / bgr8 / mono8)
Publishes   /cmd_vel           (geometry_msgs/Twist: linear.x m/s, angular.z rad/s)
            /line_follower/debug  (sensor_msgs/Image, bgr8, only if publish_debug)
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .lane_tracker import (CameraGeometry, ControlParams, LineFollower, TrackerParams,
                           draw_debug)


def _declare_from(node, prefix, obj):
    """Declare one ROS parameter per dataclass field and read it back."""
    for name, default in vars(obj).items():
        setattr(obj, name, node.declare_parameter(f"{prefix}.{name}", default).value)
    return obj


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower")
        tp = _declare_from(self, "tracker", TrackerParams())
        cp = _declare_from(self, "control", ControlParams())
        cam = _declare_from(self, "camera", CameraGeometry())
        self.enabled = self.declare_parameter("enabled", True).value
        self.publish_debug = self.declare_parameter("publish_debug", True).value

        self.follower = LineFollower(tp, cp, cam)
        self.last_stamp = None
        self.stats_period = self.declare_parameter("stats_period", 5.0).value
        self._reset_stats(None)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.dbg_pub = self.create_publisher(Image, "/line_follower/debug", 1)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"line follower up: v_max={cp.v_max} m/s, scan rows {tp.near_row}/{tp.far_row}")

    @staticmethod
    def _to_numpy(msg):
        buf = np.frombuffer(msg.data, np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            # luma; channel order only matters for the debug image
            r, g, b = (img[..., 0], img[..., 1], img[..., 2]) if msg.encoding == "rgb8" \
                else (img[..., 2], img[..., 1], img[..., 0])
            r, g, b = r.astype(np.uint16), g.astype(np.uint16), b.astype(np.uint16)
            gray = ((77 * r + 150 * g + 29 * b) >> 8).astype(np.uint8)
            bgr = img[..., ::-1] if msg.encoding == "rgb8" else img
            return gray, bgr
        if msg.encoding == "mono8":
            gray = buf.reshape(msg.height, msg.step)[:, :msg.width]
            return gray, np.repeat(gray[..., None], 3, axis=2)
        raise ValueError(f"unsupported encoding {msg.encoding}")

    def _reset_stats(self, stamp):
        self.st_t0, self.st_wall0 = stamp, time.monotonic()
        self.st_frames, self.st_age, self.st_age_max, self.st_proc = 0, 0.0, 0.0, 0.0

    def _stats(self, stamp, wall_start):
        """Every few seconds: camera rate and delays as the controller sees them.

        The controller needs roughly 30 fps and under 0.1 s from image to command;
        tools/fast_sim.py --fps/--latency shows what happens when it gets less.
        """
        if self.st_t0 is None or stamp < self.st_t0:
            self._reset_stats(stamp)
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        age = max(0.0, now - stamp)                       # sim seconds since the image was taken
        self.st_frames += 1
        self.st_age += age
        self.st_age_max = max(self.st_age_max, age)
        self.st_proc += time.monotonic() - wall_start
        sim_span = stamp - self.st_t0
        if sim_span >= self.stats_period:
            wall_span = time.monotonic() - self.st_wall0
            n = self.st_frames
            self.get_logger().info(
                f"camera {n / sim_span:.1f} fps (sim time), image age avg "
                f"{1000 * self.st_age / n:.0f} ms max {1000 * self.st_age_max:.0f} ms, "
                f"processing {1000 * self.st_proc / n:.1f} ms/frame, "
                f"real-time factor {sim_span / max(wall_span, 1e-6):.2f}")
            self._reset_stats(stamp)

    def on_image(self, msg):
        wall_start = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp < self.last_stamp - 0.5:
            self.follower = LineFollower(self.follower.tp, self.follower.cp, self.follower.cam)
            self.get_logger().info("sim time went backwards (world reset): follower restarted")
        dt = 1.0 / 30 if self.last_stamp is None else max(1e-3, stamp - self.last_stamp)
        self.last_stamp = stamp
        try:
            gray, bgr = self._to_numpy(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        speed, steer = self.follower.step(gray, dt)
        cmd = Twist()
        if self.enabled:
            cmd.linear.x = float(speed)
            cmd.angular.z = float(self.follower.yaw_rate())
        self.cmd_pub.publish(cmd)
        self._stats(stamp, wall_start)

        if self.publish_debug and self.dbg_pub.get_subscription_count() > 0:
            dbg = np.ascontiguousarray(draw_debug(bgr, self.follower))
            out = Image()
            out.header = msg.header
            out.height, out.width = dbg.shape[:2]
            out.encoding, out.step = "bgr8", dbg.shape[1] * 3
            out.data = dbg.tobytes()
            self.dbg_pub.publish(out)

    def stop(self):
        # On Ctrl-C rclpy may already have shut the context down; then there is
        # nobody left to tell, so skip the final zero command instead of crashing.
        if rclpy.ok():
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
