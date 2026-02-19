#!/usr/bin/env python3

import threading
import time
from collections import deque
from typing import Deque, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from go2_msgs.msg import QDq, MpcForces, LocomotionCmd


class QdqPlotter(Node):
    def __init__(self) -> None:
        super().__init__("qdq_plotter")

        self.declare_parameter("qdq_topic", "/qdq")
        self.declare_parameter("qdq_est_topic", "/qdq_est")
        self.declare_parameter("plot_qdq_est", False)
        self.declare_parameter("mpc_forces_topic", "/mpc_forces")
        self.declare_parameter("plot_mpc_grf", True)
        self.declare_parameter("locomotion_cmd_topic", "/locomotion_cmd_state")
        self.declare_parameter("plot_cmd", True)
        self.declare_parameter("grf_window_sec", 10.0)
        self.declare_parameter("history_sec", 0.0)
        self.declare_parameter("refresh_hz", 10.0)
        self.declare_parameter("z_pos_des_body", 0.27)

        self.qdq_topic = str(self.get_parameter("qdq_topic").value)
        self.qdq_est_topic = str(self.get_parameter("qdq_est_topic").value)
        self.plot_qdq_est = bool(self.get_parameter("plot_qdq_est").value)
        self.mpc_forces_topic = str(self.get_parameter("mpc_forces_topic").value)
        self.plot_mpc_grf = bool(self.get_parameter("plot_mpc_grf").value)
        self.locomotion_cmd_topic = str(self.get_parameter("locomotion_cmd_topic").value)
        self.plot_cmd = bool(self.get_parameter("plot_cmd").value)
        self.grf_window_sec = float(self.get_parameter("grf_window_sec").value)
        self.history_sec = float(self.get_parameter("history_sec").value)
        self.refresh_hz = float(self.get_parameter("refresh_hz").value)
        self.z_pos_des_body = float(self.get_parameter("z_pos_des_body").value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(QDq, self.qdq_topic, self.on_msg, qos)
        self.sub_est = None
        if self.plot_qdq_est:
            self.sub_est = self.create_subscription(
                QDq, self.qdq_est_topic, self.on_msg_est, qos
            )
        self.sub_mpc = None
        if self.plot_mpc_grf:
            self.sub_mpc = self.create_subscription(
                MpcForces, self.mpc_forces_topic, self.on_mpc_forces, qos
            )
        self.sub_cmd = None
        if self.plot_cmd:
            self.sub_cmd = self.create_subscription(
                LocomotionCmd, self.locomotion_cmd_topic, self.on_cmd, qos
            )

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
        self._t_est: Deque[float] = deque()
        self._q_est: Tuple[Deque[float], Deque[float], Deque[float]] = (
            deque(),
            deque(),
            deque(),
        )
        self._dq_est: Tuple[Deque[float], Deque[float], Deque[float]] = (
            deque(),
            deque(),
            deque(),
        )
        self._t_grf: Deque[float] = deque()
        self._grf_z: Tuple[Deque[float], Deque[float], Deque[float], Deque[float]] = (
            deque(),
            deque(),
            deque(),
            deque(),
        )
        self._t0_cmd = None
        self._t_cmd: Deque[float] = deque()
        self._cmd_x: Deque[float] = deque()
        self._cmd_y: Deque[float] = deque()
        self._cmd_z: Deque[float] = deque()
        self._cmd_yaw: Deque[float] = deque()
        self._cmd_gait: Deque[float] = deque()

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

    def on_msg_est(self, msg: QDq) -> None:
        t = float(msg.sim_time)
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            t_rel = t - self._t0
            self._t_est.append(t_rel)
            self._q_est[0].append(float(msg.q[0]))
            self._q_est[1].append(float(msg.q[1]))
            self._q_est[2].append(float(msg.q[2]))
            self._dq_est[0].append(float(msg.dq[0]))
            self._dq_est[1].append(float(msg.dq[1]))
            self._dq_est[2].append(float(msg.dq[2]))

            if self.history_sec > 0.0:
                while self._t_est and (self._t_est[-1] - self._t_est[0] > self.history_sec):
                    self._t_est.popleft()
                    for dq_buf in self._dq_est:
                        dq_buf.popleft()
                    for q_buf in self._q_est:
                        q_buf.popleft()

    def on_mpc_forces(self, msg: MpcForces) -> None:
        t = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            t_rel = t - self._t0
            self._t_grf.append(t_rel)

            forces = list(msg.forces)
            # Expect 12 values: [FLx,FLy,FLz, FRx,FRy,FRz, RLx,RLy,RLz, RRx,RRy,RRz]
            if len(forces) < 12:
                forces = forces + [0.0] * (12 - len(forces))
            fz = [forces[2], forces[5], forces[8], forces[11]]
            for i in range(4):
                self._grf_z[i].append(float(fz[i]))

            if self.history_sec > 0.0:
                while self._t_grf and (self._t_grf[-1] - self._t_grf[0] > self.history_sec):
                    self._t_grf.popleft()
                    for f_buf in self._grf_z:
                        f_buf.popleft()

    def on_cmd(self, msg: LocomotionCmd) -> None:
        t = time.monotonic()
        with self._lock:
            if self._t0_cmd is None:
                self._t0_cmd = t
            t_rel = t - self._t0_cmd
            self._t_cmd.append(t_rel)
            self._cmd_x.append(float(msg.x_vel))
            self._cmd_y.append(float(msg.y_vel))
            self._cmd_z.append(float(msg.z_pos))
            self._cmd_yaw.append(float(msg.yaw_rate))
            self._cmd_gait.append(float(msg.gait_hz))

            if self.history_sec > 0.0:
                while self._t_cmd and (self._t_cmd[-1] - self._t_cmd[0] > self.history_sec):
                    self._t_cmd.popleft()
                    self._cmd_x.popleft()
                    self._cmd_y.popleft()
                    self._cmd_z.popleft()
                    self._cmd_yaw.popleft()
                    self._cmd_gait.popleft()

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
        lines_q_est = []
        if self.plot_qdq_est:
            lines_q_est = [
                ax_q[0].plot([], [], linestyle="--", label="qdq_est q[0]")[0],
                ax_q[0].plot([], [], linestyle="--", label="qdq_est q[1]")[0],
                ax_q[1].plot([], [], linestyle="--", label="qdq_est q[2]")[0],
            ]
        z_des_line = ax_q[1].axhline(
            self.z_pos_des_body, linestyle="--", color="tab:gray", label="z_des"
        )
        lines_dq = [
            ax_dq[0].plot([], [], label="dq[0]")[0],
            ax_dq[0].plot([], [], label="dq[1]")[0],
            ax_dq[1].plot([], [], label="dq[2]")[0],
        ]
        lines_dq_est = []
        if self.plot_qdq_est:
            lines_dq_est = [
                ax_dq[0].plot([], [], linestyle="--", label="qdq_est dq[0]")[0],
                ax_dq[0].plot([], [], linestyle="--", label="qdq_est dq[1]")[0],
                ax_dq[1].plot([], [], linestyle="--", label="qdq_est dq[2]")[0],
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
                t_est = list(self._t_est)
                q0e, q1e, q2e = map(list, self._q_est)
                dq0e, dq1e, dq2e = map(list, self._dq_est)

            if not t:
                return lines_q + lines_q_est + lines_dq + lines_dq_est

            lines_q[0].set_data(t, q0)
            lines_q[1].set_data(t, q1)
            lines_q[2].set_data(t, q2)
            lines_dq[0].set_data(t, dq0)
            lines_dq[1].set_data(t, dq1)
            lines_dq[2].set_data(t, dq2)

            if self.plot_qdq_est and t_est:
                lines_q_est[0].set_data(t_est, q0e)
                lines_q_est[1].set_data(t_est, q1e)
                lines_q_est[2].set_data(t_est, q2e)
                lines_dq_est[0].set_data(t_est, dq0e)
                lines_dq_est[1].set_data(t_est, dq1e)
                lines_dq_est[2].set_data(t_est, dq2e)

            # Keep left bound at 0, extend right bound to latest time.
            t_max = max(t + t_est) if t_est else max(t)
            for i in range(2):
                ax_q[i].set_xlim(0.0, max(t_max, 1e-3))
                ax_dq[i].set_xlim(0.0, max(t_max, 1e-3))

                ax_q[i].relim()
                ax_q[i].autoscale_view(scalex=False, scaley=False)
                ax_dq[i].relim()
                ax_dq[i].autoscale_view(scalex=False, scaley=False)

            return lines_q + lines_q_est + lines_dq + lines_dq_est

        interval_ms = int(1000.0 / max(self.refresh_hz, 1.0))
        # Keep a reference to the animation to avoid garbage collection.
        self._anim = FuncAnimation(fig, update, interval=interval_ms)

        if self.plot_cmd:
            fig_cmd, axes_cmd = plt.subplots(2, 2, sharex=True)
            fig_cmd.suptitle("Locomotion Cmd")

            line_cmd_xy = axes_cmd[0][0].plot([], [], label="x_vel")[0]
            line_cmd_yy = axes_cmd[0][0].plot([], [], label="y_vel")[0]
            line_cmd_z = axes_cmd[0][1].plot([], [], label="z_pos")[0]
            line_cmd_yaw = axes_cmd[1][0].plot([], [], label="yaw_rate")[0]
            line_cmd_gait = axes_cmd[1][1].plot([], [], label="gait_hz")[0]

            axes_cmd[0][0].set_ylabel("vel (m/s)")
            axes_cmd[0][0].legend(loc="upper right")
            axes_cmd[0][1].set_ylabel("z (m)")
            axes_cmd[0][1].set_ylim(0.0, 0.5)
            axes_cmd[0][1].legend(loc="upper right")
            axes_cmd[1][0].set_ylabel("yaw (rad/s)")
            axes_cmd[1][0].set_xlabel("time (s)")
            axes_cmd[1][0].legend(loc="upper right")
            axes_cmd[1][1].set_ylabel("gait (Hz)")
            axes_cmd[1][1].set_xlabel("time (s)")
            axes_cmd[1][1].set_ylim(0.0, 5.0)
            axes_cmd[1][1].legend(loc="upper right")

            def update_cmd(_):
                with self._lock:
                    t = list(self._t_cmd)
                    x = list(self._cmd_x)
                    y = list(self._cmd_y)
                    z = list(self._cmd_z)
                    yaw = list(self._cmd_yaw)
                    gait = list(self._cmd_gait)

                if not t:
                    return [line_cmd_xy, line_cmd_yy, line_cmd_z, line_cmd_yaw, line_cmd_gait]

                line_cmd_xy.set_data(t, x)
                line_cmd_yy.set_data(t, y)
                line_cmd_z.set_data(t, z)
                line_cmd_yaw.set_data(t, yaw)
                line_cmd_gait.set_data(t, gait)

                t_max = max(t)
                t_min = t_max - 10.0
                for ax in (axes_cmd[0][0], axes_cmd[0][1], axes_cmd[1][0], axes_cmd[1][1]):
                    ax.set_xlim(t_min, max(t_max, t_min + 1e-3))
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)

                return [line_cmd_xy, line_cmd_yy, line_cmd_z, line_cmd_yaw, line_cmd_gait]

            self._anim_cmd = FuncAnimation(fig_cmd, update_cmd, interval=interval_ms)

        if self.plot_mpc_grf:
            fig_grf, axes_grf = plt.subplots(2, 2, sharex=True)
            fig_grf.suptitle("MPC Desired GRF (Fz per leg)")
            leg_names = ["FL", "FR", "RL", "RR"]
            grf_lines = []
            for i in range(4):
                ax = axes_grf[i // 2][i % 2]
                ax.set_ylabel("Fz (N)")
                ax.set_xlabel("time (s)")
                line = ax.plot([], [], label=f"{leg_names[i]} Fz")[0]
                ax.legend(loc="upper right")
                grf_lines.append(line)

            def update_grf(_):
                with self._lock:
                    t = list(self._t_grf)
                    fz = [list(buf) for buf in self._grf_z]

                if not t:
                    return grf_lines

                for i in range(4):
                    grf_lines[i].set_data(t, fz[i])
                    t_max = max(t)
                    t_min = t_max - max(self.grf_window_sec, 0.1)
                    axes_grf[i // 2][i % 2].set_xlim(t_min, t_max)
                    axes_grf[i // 2][i % 2].relim()
                    axes_grf[i // 2][i % 2].autoscale_view(scalex=False, scaley=True)

                return grf_lines

            self._anim_grf = FuncAnimation(fig_grf, update_grf, interval=interval_ms)
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
