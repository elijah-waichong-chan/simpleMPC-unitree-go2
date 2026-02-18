#!/usr/bin/env python3

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from go2_msgs.msg import QDq
from std_msgs.msg import Bool


class QdqEstBridge(Node):
    def __init__(self) -> None:
        super().__init__("qdq_est_bridge")

        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("qdq_topic", "/qdq_est")

        odom_topic = self.get_parameter("odom_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        qdq_topic = self.get_parameter("qdq_topic").value

        self.joint_order: List[str] = [
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        ]

        self._last_joint_names: Optional[List[str]] = None
        self._joint_index: Dict[str, int] = {}
        self._missing_warned = False

        self._last_odom: Optional[Odometry] = None
        self._last_joint: Optional[JointState] = None
        self._standing_ready = False
        self._standing_wait_logged = False

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.pub_qdq = self.create_publisher(QDq, qdq_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.on_odom, sensor_qos)
        self.sub_joint = self.create_subscription(JointState, joint_states_topic, self.on_joint, sensor_qos)
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub_standing = self.create_subscription(
            Bool, "/status/standing_init", self.on_standing_status, status_qos
        )


    def on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self.try_publish()

    def on_joint(self, msg: JointState) -> None:
        self._last_joint = msg
        self._update_joint_index(msg.name)
        # Publish only on odom callbacks to keep a single stream rate.

    def on_standing_status(self, msg: Bool) -> None:
        self._standing_ready = bool(msg.data)

    def _update_joint_index(self, names: List[str]) -> None:
        if self._last_joint_names == list(names):
            return
        self._last_joint_names = list(names)
        self._joint_index = {name: i for i, name in enumerate(names)}
        missing = [n for n in self.joint_order if n not in self._joint_index]
        if missing and not self._missing_warned:
            self.get_logger().warn(
                f"JointState missing expected joints: {missing}. "
                "qdq_est will publish zeros for missing joints."
            )
            self._missing_warned = True

    def try_publish(self) -> None:
        if not self._standing_ready:
            if not self._standing_wait_logged:
                self.get_logger().info("qdq_est_bridge waiting for /status/standing_init...")
                self._standing_wait_logged = True
            return
        if self._last_odom is None or self._last_joint is None:
            return

        odom = self._last_odom
        js = self._last_joint

        msg = QDq()

        # sim_time from odom header
        stamp = odom.header.stamp
        msg.sim_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        # Base pose
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation
        msg.q[0] = pos.x
        msg.q[1] = pos.y
        msg.q[2] = pos.z
        # QDq expects quaternion as (qw, qx, qy, qz)
        msg.q[3] = ori.w
        msg.q[4] = ori.x
        msg.q[5] = ori.y
        msg.q[6] = ori.z

        # Base twist: convert base-frame linear velocity to world frame
        lin = odom.twist.twist.linear
        ang = odom.twist.twist.angular
        qw, qx, qy, qz = ori.w, ori.x, ori.y, ori.z
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        vx_b = lin.x
        vy_b = lin.y
        vz_b = lin.z
        vx_w = r00 * vx_b + r01 * vy_b + r02 * vz_b
        vy_w = r10 * vx_b + r11 * vy_b + r12 * vz_b
        vz_w = r20 * vx_b + r21 * vy_b + r22 * vz_b

        msg.dq[0] = vx_w
        msg.dq[1] = vy_w
        msg.dq[2] = vz_w
        msg.dq[3] = ang.x
        msg.dq[4] = ang.y
        msg.dq[5] = ang.z

        # Joint positions/velocities
        for i, name in enumerate(self.joint_order):
            j_idx = self._joint_index.get(name, None)
            if j_idx is None:
                msg.q[7 + i] = 0.0
                msg.dq[6 + i] = 0.0
                continue

            if j_idx < len(js.position):
                msg.q[7 + i] = js.position[j_idx]
            if j_idx < len(js.velocity):
                msg.dq[6 + i] = js.velocity[j_idx]

        self.pub_qdq.publish(msg)



def main(args=None) -> None:
    rclpy.init(args=args)
    node = QdqEstBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
