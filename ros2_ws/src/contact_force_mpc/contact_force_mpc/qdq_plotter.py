#!/usr/bin/env python3

import threading
from collections import deque
from typing import Deque, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from go2_msgs.msg import QDq


class QdqPlotter(Node):
    def __init__(self) -> None:
        super().__init__("qdq_plotter")

        self.declare_parameter("qdq_topic", "/qdq")
        self.declare_parameter("history_sec", 0.0)
        self.declare_parameter("refresh_hz", 10.0)

        self.qdq_topic = str(self.get_parameter("qdq_topic").value)
        self.history_sec = float(self.get_parameter("history_sec").value)
        self.refresh_hz = float(self.get_parameter("refresh_hz").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(QDq, self.qdq_topic, self.on_msg, qos)

        self._lock = threading.Lock()
        self._t0 = None
        self._t: Deque[float] = deque()
        self._q: Tuple[Deque[float], Deque[float], Deque[float]] = (
            deque(),
            deque(),
            deque(),
        )
        self._dq: Tuple[Deque[float], Deque[float], Deque[float]] = (
            deque(),
            deque(),
            deque(),
        )

    def on_msg(self, msg: QDq) -> None:
        t = float(msg.sim_time)
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            t_rel = t - self._t0
            self._t.append(t_rel)
            self._q[0].append(float(msg.q[0]))
            self._q[1].append(float(msg.q[1]))
            self._q[2].append(float(msg.q[2]))
            self._dq[0].append(float(msg.dq[0]))
            self._dq[1].append(float(msg.dq[1]))
            self._dq[2].append(float(msg.dq[2]))

            # Trim history by time (0.0 = keep all)
            if self.history_sec > 0.0:
                while self._t and (self._t[-1] - self._t[0] > self.history_sec):
                    self._t.popleft()
                    for dq_buf in self._dq:
                        dq_buf.popleft()
                    for q_buf in self._q:
                        q_buf.popleft()

    def run_plot(self) -> None:
        fig, axes = plt.subplots(2, 2, sharex="col")
        fig.suptitle("QDq (world position & world linear velocity)")

        ax_q = axes[0]
        ax_dq = axes[1]

        lines_q = [
            ax_q[0].plot([], [], label="q[0]")[0],
            ax_q[0].plot([], [], label="q[1]")[0],
            ax_q[1].plot([], [], label="q[2]")[0],
        ]
        lines_dq = [
            ax_dq[0].plot([], [], label="dq[0]")[0],
            ax_dq[0].plot([], [], label="dq[1]")[0],
            ax_dq[1].plot([], [], label="dq[2]")[0],
        ]

        ax_q[0].set_ylabel("pos (m)")
        ax_q[0].set_ylim(-5.0, 5.0)
        ax_q[0].legend(loc="upper right")
        ax_q[1].set_ylabel("pos (m)")
        ax_q[1].set_ylim(0.0, 0.5)
        ax_q[1].legend(loc="upper right")

        ax_dq[0].set_ylabel("vel (m/s)")
        ax_dq[0].set_ylim(-1.0, 1.0)
        ax_dq[0].set_xlabel("time (s)")
        ax_dq[0].legend(loc="upper right")
        ax_dq[1].set_ylabel("vel (m/s)")
        ax_dq[1].set_ylim(-1.0, 1.0)
        ax_dq[1].set_xlabel("time (s)")
        ax_dq[1].legend(loc="upper right")

        def update(_):
            with self._lock:
                t = list(self._t)
                q0, q1, q2 = map(list, self._q)
                dq0, dq1, dq2 = map(list, self._dq)

            if not t:
                return lines_q + lines_dq

            lines_q[0].set_data(t, q0)
            lines_q[1].set_data(t, q1)
            lines_q[2].set_data(t, q2)
            lines_dq[0].set_data(t, dq0)
            lines_dq[1].set_data(t, dq1)
            lines_dq[2].set_data(t, dq2)

            # Keep left bound at 0, extend right bound to latest time.
            t_max = max(t)
            for i in range(2):
                ax_q[i].set_xlim(0.0, max(t_max, 1e-3))
                ax_dq[i].set_xlim(0.0, max(t_max, 1e-3))

                ax_q[i].relim()
                ax_q[i].autoscale_view(scalex=False, scaley=False)
                ax_dq[i].relim()
                ax_dq[i].autoscale_view(scalex=False, scaley=False)

            return lines_q + lines_dq

        interval_ms = int(1000.0 / max(self.refresh_hz, 1.0))
        # Keep a reference to the animation to avoid garbage collection.
        self._anim = FuncAnimation(fig, update, interval=interval_ms)
        plt.show()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QdqPlotter()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_plot()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
