#!/usr/bin/env python3

import time
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from sensor_msgs.msg import Joy
from go2_msgs.msg import LocomotionCmd


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class JoyLocomotionBridge(Node):
    def __init__(self) -> None:
        super().__init__("xbox_controller_bridge")

        # Topics
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("locomotion_cmd_topic", "/locomotion_cmd")

        # Publish settings
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter("deadzone", 0.05)

        # Axis mapping
        self.declare_parameter("axis_x", 1)
        self.declare_parameter("axis_y", 2)
        self.declare_parameter("axis_yaw", 0)
        self.declare_parameter("axis_z", 3)

        # Scales (normal)
        self.declare_parameter("scale_x", 0.5)
        self.declare_parameter("scale_y", 0.2)
        self.declare_parameter("scale_yaw", 2.0)
        self.declare_parameter("scale_z_rate", 0.05)

        # Turbo
        self.declare_parameter("enable_turbo_button", 2)
        self.declare_parameter("scale_x_turbo", 1.0)
        self.declare_parameter("scale_y_turbo", 0.4)
        self.declare_parameter("scale_yaw_turbo", 4.0)
        self.declare_parameter("scale_z_rate_turbo", 0.1)

        # Z position and gait settings
        self.declare_parameter("z_pos_init", 0.27)
        self.declare_parameter("z_pos_min", 0.15)
        self.declare_parameter("z_pos_max", 0.45)
        self.declare_parameter("gait_hz_init", 3.0)
        self.declare_parameter("gait_hz_min", 0.0)
        self.declare_parameter("gait_hz_max", 4.0)
        self.declare_parameter("gait_hz_step", 0.25)

        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.cmd_topic = str(self.get_parameter("locomotion_cmd_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.axis_x = int(self.get_parameter("axis_x").value)
        self.axis_y = int(self.get_parameter("axis_y").value)
        self.axis_yaw = int(self.get_parameter("axis_yaw").value)
        self.axis_z = int(self.get_parameter("axis_z").value)

        self.scale_x = float(self.get_parameter("scale_x").value)
        self.scale_y = float(self.get_parameter("scale_y").value)
        self.scale_yaw = float(self.get_parameter("scale_yaw").value)
        self.scale_z_rate = float(self.get_parameter("scale_z_rate").value)

        self.turbo_button = int(self.get_parameter("enable_turbo_button").value)
        self.scale_x_turbo = float(self.get_parameter("scale_x_turbo").value)
        self.scale_y_turbo = float(self.get_parameter("scale_y_turbo").value)
        self.scale_yaw_turbo = float(self.get_parameter("scale_yaw_turbo").value)
        self.scale_z_rate_turbo = float(self.get_parameter("scale_z_rate_turbo").value)

        self.z_pos = float(self.get_parameter("z_pos_init").value)
        self.z_pos_min = float(self.get_parameter("z_pos_min").value)
        self.z_pos_max = float(self.get_parameter("z_pos_max").value)
        self.gait_hz = float(self.get_parameter("gait_hz_init").value)
        self.gait_hz_min = float(self.get_parameter("gait_hz_min").value)
        self.gait_hz_max = float(self.get_parameter("gait_hz_max").value)
        self.gait_hz_step = float(self.get_parameter("gait_hz_step").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.sub_joy = self.create_subscription(Joy, self.joy_topic, self.on_joy, qos)
        self.pub_cmd = self.create_publisher(LocomotionCmd, self.cmd_topic, qos)

        self._axes: List[float] = []
        self._buttons: List[int] = []
        self._last_buttons: List[int] = []
        self._last_joy_time = None
        self._last_pub_time = time.monotonic()

        period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= len(self._axes):
            return 0.0
        v = float(self._axes[idx])
        if abs(v) < self.deadzone:
            return 0.0
        return v

    def _button(self, idx: int) -> bool:
        if idx < 0 or idx >= len(self._buttons):
            return False
        return self._buttons[idx] != 0

    def on_joy(self, msg: Joy) -> None:
        self._axes = list(msg.axes)
        self._buttons = list(msg.buttons)
        self._last_joy_time = time.monotonic()

        self._last_buttons = list(self._buttons)

    def on_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_pub_time
        self._last_pub_time = now

        stale = self._last_joy_time is None or (now - self._last_joy_time) > self.cmd_timeout_s

        turbo = self._button(self.turbo_button)
        sx = self.scale_x_turbo if turbo else self.scale_x
        sy = self.scale_y_turbo if turbo else self.scale_y
        syaw = self.scale_yaw_turbo if turbo else self.scale_yaw
        sz = self.scale_z_rate_turbo if turbo else self.scale_z_rate

        if stale:
            x_vel = 0.0
            y_vel = 0.0
            yaw_rate = 0.0
            z_rate = 0.0
        else:
            x_vel = self._axis(self.axis_x) * sx
            y_vel = self._axis(self.axis_y) * sy
            yaw_rate = self._axis(self.axis_yaw) * syaw
            z_rate = self._axis(self.axis_z) * sz

        if dt > 0.0:
            self.z_pos = _clamp(self.z_pos + z_rate * dt, self.z_pos_min, self.z_pos_max)

        msg = LocomotionCmd()
        msg.stamp = self.get_clock().now().to_msg()
        msg.x_vel = float(x_vel)
        msg.y_vel = float(y_vel)
        msg.z_pos = float(self.z_pos)
        msg.yaw_rate = float(yaw_rate)
        msg.gait_hz = float(self.gait_hz)
        self.pub_cmd.publish(msg)


def main() -> None:
    rclpy.init()
    node = JoyLocomotionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
