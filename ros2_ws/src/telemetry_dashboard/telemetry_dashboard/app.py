#!/usr/bin/env python3

import os
import threading
import time
import uuid
import io
from collections import deque
from typing import Deque, Dict, List, Tuple

import numpy as np
import matplotlib.pyplot as plt
import streamlit as st
import streamlit.components.v1 as components

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

from go2_msgs.msg import QDq, MpcForces, LocomotionCmd
from sensor_msgs.msg import Joy
from unitree_go.msg import LowState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def quat_to_rpy(qw: float, qx: float, qy: float, qz: float) -> Tuple[float, float, float]:
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * (np.pi / 2.0)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TelemetryNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.qdq_topic = self.declare_parameter("qdq_topic", "/qdq").value
        self.qdq_est_topic = self.declare_parameter("qdq_est_topic", "/qdq_est").value
        self.cmd_topic = self.declare_parameter("locomotion_cmd_topic", "/locomotion_cmd_state").value
        self.mpc_forces_topic = self.declare_parameter("mpc_forces_topic", "/mpc_forces").value
        self.history_sec = float(self.declare_parameter("history_sec", 20.0).value)
        self.max_samples = int(self.declare_parameter("max_samples", 6000).value)
        self.max_plot_points = int(self.declare_parameter("max_plot_points", 1000).value)
        self.status_timeout_s = float(self.declare_parameter("status_timeout_s", 3.0).value)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._lock = threading.Lock()
        self._t0_q = None
        self._t0_cmd = None

        self._t_q: Deque[float] = deque(maxlen=self.max_samples)
        self._q_xyz = [deque(maxlen=self.max_samples) for _ in range(3)]
        self._dq_xyz = [deque(maxlen=self.max_samples) for _ in range(3)]
        self._rpy = [deque(maxlen=self.max_samples) for _ in range(3)]
        self._dq_joints = [deque(maxlen=self.max_samples) for _ in range(12)]

        self._t_q_est: Deque[float] = deque(maxlen=self.max_samples)
        self._q_xyz_est = [deque(maxlen=self.max_samples) for _ in range(3)]
        self._dq_xyz_est = [deque(maxlen=self.max_samples) for _ in range(3)]
        self._rpy_est = [deque(maxlen=self.max_samples) for _ in range(3)]

        self._t_cmd: Deque[float] = deque(maxlen=self.max_samples)
        self._cmd_x = deque(maxlen=self.max_samples)
        self._cmd_y = deque(maxlen=self.max_samples)
        self._cmd_z = deque(maxlen=self.max_samples)
        self._cmd_yaw = deque(maxlen=self.max_samples)
        self._cmd_gait = deque(maxlen=self.max_samples)

        self._t_grf: Deque[float] = deque(maxlen=self.max_samples)
        self._grf_z = [deque(maxlen=self.max_samples) for _ in range(4)]

        self._status: Dict[str, Tuple[bool, float]] = {}
        self._topic_available: Dict[str, bool] = {}
        self._topic_rate: Dict[str, float] = {}
        self._topic_last_time: Dict[str, float] = {}
        self._topic_names = {
            "qdq": str(self.qdq_topic),
            "qdq_est": str(self.qdq_est_topic),
            "mpc_forces": "/mpc_forces",
            "lowstate": "/lowstate",
            "joy": "/joy",
        }

        self.create_subscription(QDq, str(self.qdq_topic), self.on_qdq, qos)
        self.create_subscription(QDq, str(self.qdq_est_topic), self.on_qdq_est, qos)
        self.create_subscription(LocomotionCmd, str(self.cmd_topic), self.on_cmd, qos)
        self.create_subscription(MpcForces, str(self.mpc_forces_topic), self.on_grf, qos)
        self.create_subscription(LowState, "/lowstate", self.on_lowstate, qos)
        self.create_subscription(Joy, "/joy", self.on_joy, qos)

        self.create_subscription(Bool, "/status/mpc/is_running",
                                 lambda m: self.on_status("mpc", m), status_qos)
        self.create_subscription(Bool, "/status/inekf/is_running",
                                 lambda m: self.on_status("inekf", m), status_qos)
        self.create_subscription(Bool, "/status/loco_ctrl/is_running",
                                 lambda m: self.on_status("loco_ctrl", m), status_qos)
        self.create_subscription(Bool, "/status/mujoco/is_running",
                                 lambda m: self.on_status("mujoco", m), status_qos)
        self.create_subscription(Bool, "/status/loco_ctrl/safety_stop",
                                 lambda m: self.on_status("safety_stop", m), status_qos)
        self.create_subscription(Bool, "/status/standing_init",
                                 lambda m: self.on_status("standing_init", m), status_qos)

        self.topic_timer = self.create_timer(1.0, self._update_topic_availability)

    def on_status(self, name: str, msg: Bool) -> None:
        with self._lock:
            self._status[name] = (bool(msg.data), time.monotonic())

    def on_qdq(self, msg: QDq) -> None:
        t = time.monotonic()
        with self._lock:
            self._status["qdq"] = (True, t)
            self._update_topic_rate("qdq", t)
            if self._t0_q is None:
                self._t0_q = t
            t_rel = t - self._t0_q
            self._t_q.append(t_rel)
            self._q_xyz[0].append(float(msg.q[0]))
            self._q_xyz[1].append(float(msg.q[1]))
            self._q_xyz[2].append(float(msg.q[2]))
            roll, pitch, yaw = quat_to_rpy(msg.q[3], msg.q[4], msg.q[5], msg.q[6])
            self._rpy[0].append(float(roll))
            self._rpy[1].append(float(pitch))
            self._rpy[2].append(float(yaw))
            self._dq_xyz[0].append(float(msg.dq[0]))
            self._dq_xyz[1].append(float(msg.dq[1]))
            self._dq_xyz[2].append(float(msg.dq[2]))
            for i in range(12):
                self._dq_joints[i].append(float(msg.dq[6 + i]))

    def on_qdq_est(self, msg: QDq) -> None:
        t = time.monotonic()
        with self._lock:
            self._status["qdq_est"] = (True, t)
            self._update_topic_rate("qdq_est", t)
            if self._t0_q is None:
                self._t0_q = t
            t_rel = t - self._t0_q
            self._t_q_est.append(t_rel)
            self._q_xyz_est[0].append(float(msg.q[0]))
            self._q_xyz_est[1].append(float(msg.q[1]))
            self._q_xyz_est[2].append(float(msg.q[2]))
            roll, pitch, yaw = quat_to_rpy(msg.q[3], msg.q[4], msg.q[5], msg.q[6])
            self._rpy_est[0].append(float(roll))
            self._rpy_est[1].append(float(pitch))
            self._rpy_est[2].append(float(yaw))
            self._dq_xyz_est[0].append(float(msg.dq[0]))
            self._dq_xyz_est[1].append(float(msg.dq[1]))
            self._dq_xyz_est[2].append(float(msg.dq[2]))

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

    def on_grf(self, msg: MpcForces) -> None:
        t = time.monotonic()
        with self._lock:
            self._status["mpc_forces"] = (True, t)
            self._update_topic_rate("mpc_forces", t)
            if self._t0_q is None:
                self._t0_q = t
            t_rel = t - self._t0_q
            self._t_grf.append(t_rel)
            forces = list(msg.forces)
            if len(forces) < 12:
                forces += [0.0] * (12 - len(forces))
            fz = [forces[2], forces[5], forces[8], forces[11]]
            for i in range(4):
                self._grf_z[i].append(float(fz[i]))

    def on_lowstate(self, msg: LowState) -> None:
        with self._lock:
            self._status["lowstate"] = (True, time.monotonic())
            self._update_topic_rate("lowstate", time.monotonic())

    def on_joy(self, msg: Joy) -> None:
        with self._lock:
            t = time.monotonic()
            self._status["joy"] = (True, t)
            self._update_topic_rate("joy", t)

    def _update_topic_rate(self, key: str, t: float) -> None:
        last_t = self._topic_last_time.get(key)
        if last_t is not None:
            dt = t - last_t
            if dt > 1e-6:
                inst = 1.0 / dt
                prev = self._topic_rate.get(key, inst)
                # Light smoothing to avoid flicker.
                self._topic_rate[key] = 0.8 * prev + 0.2 * inst
        self._topic_last_time[key] = t

    def _update_topic_availability(self) -> None:
        try:
            topics = self.get_topic_names_and_types()
            available = {name for name, _types in topics}
            with self._lock:
                for _key, name in self._topic_names.items():
                    self._topic_available[name] = name in available
        except Exception:
            with self._lock:
                for _key, name in self._topic_names.items():
                    self._topic_available[name] = False

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return {
                "t_q": list(self._t_q),
                "q_xyz": [list(buf) for buf in self._q_xyz],
                "dq_xyz": [list(buf) for buf in self._dq_xyz],
                "rpy": [list(buf) for buf in self._rpy],
                "dq_joints": [list(buf) for buf in self._dq_joints],
                "t_q_est": list(self._t_q_est),
                "q_xyz_est": [list(buf) for buf in self._q_xyz_est],
                "dq_xyz_est": [list(buf) for buf in self._dq_xyz_est],
                "rpy_est": [list(buf) for buf in self._rpy_est],
                "t_cmd": list(self._t_cmd),
                "cmd": [
                    list(self._cmd_x),
                    list(self._cmd_y),
                    list(self._cmd_z),
                    list(self._cmd_yaw),
                    list(self._cmd_gait),
                ],
                "t_grf": list(self._t_grf),
                "grf_z": [list(buf) for buf in self._grf_z],
                "status": dict(self._status),
                "topic_available": dict(self._topic_available),
                "topic_rate": dict(self._topic_rate),
                "topic_names": dict(self._topic_names),
                "status_timeout_s": self.status_timeout_s,
                "max_plot_points": self.max_plot_points,
            }


def get_ros_node() -> TelemetryNode:
    if "ros_node" in st.session_state:
        return st.session_state["ros_node"]

    try:
        if not rclpy.ok():
            rclpy.init(args=None)
    except RuntimeError:
        # rclpy may already be initialized in this process
        pass

    if "ros_node_name" not in st.session_state:
        st.session_state["ros_node_name"] = f"telemetry_dashboard_{os.getpid()}_{uuid.uuid4().hex[:6]}"

    node = TelemetryNode(st.session_state["ros_node_name"])
    st.session_state["svc_start"] = node.create_client(Trigger, "/launch/mujoco_robot/start")
    st.session_state["svc_stop"] = node.create_client(Trigger, "/launch/mujoco_robot/stop")
    st.session_state["svc_ctrl_start"] = node.create_client(Trigger, "/launch/control_stack/start")
    st.session_state["svc_ctrl_stop"] = node.create_client(Trigger, "/launch/control_stack/stop")
    st.session_state["svc_emergency_stop"] = node.create_client(Trigger, "/loco_ctrl/emergency_stop")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    st.session_state["ros_node"] = node
    return node


def _autorefresh_fallback(interval_ms: int) -> None:
    if interval_ms <= 0:
        return
    time.sleep(float(interval_ms) / 1000.0)
    if hasattr(st, "rerun"):
        st.rerun()
    elif hasattr(st, "experimental_rerun"):
        st.experimental_rerun()


def _get_fresh_status(status_map: Dict[str, Tuple[bool, float]], key: str, now: float, timeout: float):
    if key not in status_map:
        return None
    val, t = status_map[key]
    if (now - t) > timeout:
        return None
    return bool(val)


def render_status(snapshot: Dict[str, object], timeout_s: float) -> None:
    now = time.monotonic()
    timeout = float(timeout_s)
    status_map = snapshot["status"]

    labels = [
        ("mujoco", "MuJoCo"),
        ("inekf", "State Estimator"),
        ("control_stack", "Locomotion Controller"),
        ("safety_stop", "Safety Stop"),
    ]

    mpc_val = _get_fresh_status(status_map, "mpc", now, timeout)
    loco_val = _get_fresh_status(status_map, "loco_ctrl", now, timeout)
    if mpc_val is True and loco_val is True:
        control_stack_val = True
    elif mpc_val is False or loco_val is False:
        control_stack_val = False
    else:
        control_stack_val = None

    status_view = {
        "mujoco": _get_fresh_status(status_map, "mujoco", now, timeout),
        "inekf": _get_fresh_status(status_map, "inekf", now, timeout),
        "control_stack": control_stack_val,
        "safety_stop": _get_fresh_status(status_map, "safety_stop", now, timeout),
    }

    cols = st.columns(len(labels))
    for idx, (key, label) in enumerate(labels):
        with cols[idx]:
            val = status_view.get(key)
            if val is None:
                if key == "safety_stop":
                    st.success(f"{label}: OK")
                else:
                    st.error(f"{label}: not running")
                continue
            if key == "safety_stop":
                if val:
                    st.error(f"{label}: TRIGGERED")
                else:
                    st.success(f"{label}: OK")
            else:
                if val:
                    st.success(f"{label}: running")
                else:
                    if key == "control_stack":
                        st.warning(f"{label}: awaiting states")
                    elif key == "inekf":
                        st.warning(f"{label}: awaiting robot data")
                    else:
                        st.error(f"{label}: not running")


def filter_window(t: List[float], series: List[List[float]], window: float):
    if not t:
        return [], [list() for _ in series]
    t_max = t[-1]
    t_min = t_max - window
    idx0 = 0
    while idx0 < len(t) and t[idx0] < t_min:
        idx0 += 1
    t_win = t[idx0:]
    series_win = [s[idx0:] for s in series]
    return t_win, series_win


def decimate(t: List[float], series: List[List[float]], max_points: int):
    if not t or max_points <= 0:
        return t, series
    n = len(t)
    if n <= max_points:
        return t, series
    step = max(1, int(np.ceil(n / max_points)))
    t_ds = t[::step]
    series_ds = [s[::step] for s in series]
    return t_ds, series_ds


def plot_series(title: str, t: List[float], series: List[List[float]], labels: List[str], ylim=None, max_points: int = 1000):
    fig = _make_series_fig(t, series, labels, title=title, ylim=ylim, max_points=max_points)
    if fig is None:
        st.write(f"{title}: no data")
        return
    st.pyplot(fig, clear_figure=True)


def _make_series_fig(
    t: List[float],
    series: List[List[float]],
    labels: List[str],
    title: str,
    ylim=None,
    max_points: int = 1000,
):
    if not t or not series:
        return None
    n = min(len(t), *(len(s) for s in series))
    if n <= 0:
        return None
    t = [float(x) for x in t[:n]]
    fig, ax = plt.subplots()
    t, series = decimate(t, [list(s[:n]) for s in series], max_points)
    for i, label in enumerate(labels):
        y = [float(x) for x in series[i]]
        ax.plot(t, y, label=label)
    ax.set_title(title)
    ax.set_xlabel("time (s)")
    ax.legend(loc="upper right")
    if ylim is not None:
        ax.set_ylim(ylim[0], ylim[1])
    return fig


def _fig_to_png(fig) -> bytes:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=120, bbox_inches="tight")
    plt.close(fig)
    return buf.getvalue()


def _render_cached_plot(key: str, build_fig_fn, cache: Dict[str, object], update: bool, empty_message: str) -> None:
    if not update and key in cache:
        st.image(cache[key], use_container_width=True)
        return
    fig = build_fig_fn()
    if fig is None:
        cache.pop(key, None)
        st.write(empty_message)
        return
    png = _fig_to_png(fig)
    cache[key] = png
    st.image(png, use_container_width=True)


def _style_sidebar_buttons(mujoco_running: bool, locomotion_active: bool) -> None:
    start_green = "#2ecc71"
    start_border = "#27ae60"
    stop_red = "#e74c3c"
    stop_border = "#c0392b"
    gray_bg = "#e0e0e0"
    gray_border = "#cfcfcf"
    gray_text = "#777777"
    js = f"""
    <script>
    const styleBtn = (label, active, bg, border) => {{
      const btns = Array.from(window.parent.document.querySelectorAll('button'));
      const btn = btns.find(b => b.innerText.trim() === label);
      if (!btn) return false;
      if (active) {{
        btn.disabled = false;
        btn.style.setProperty('background-color', bg, 'important');
        btn.style.setProperty('color', '#ffffff', 'important');
        btn.style.setProperty('border', '1px solid ' + border, 'important');
        btn.style.setProperty('opacity', '1.0', 'important');
        btn.style.setProperty('cursor', 'pointer', 'important');
      }} else {{
        btn.disabled = true;
        btn.style.setProperty('background-color', '{gray_bg}', 'important');
        btn.style.setProperty('color', '{gray_text}', 'important');
        btn.style.setProperty('border', '1px solid {gray_border}', 'important');
        btn.style.setProperty('opacity', '0.9', 'important');
        btn.style.setProperty('cursor', 'not-allowed', 'important');
      }}
      return true;
    }};

    const apply = () => {{
      let ok = true;
      ok = styleBtn('Start MuJoCo', {str(not mujoco_running).lower()}, '{start_green}', '{start_border}') && ok;
      ok = styleBtn('Stop MuJoCo', {str(mujoco_running).lower()}, '{stop_red}', '{stop_border}') && ok;
      ok = styleBtn('Start Control Stack', {str(not locomotion_active).lower()}, '{start_green}', '{start_border}') && ok;
      ok = styleBtn('Stop Control Stack', {str(locomotion_active).lower()}, '{stop_red}', '{stop_border}') && ok;
      ok = styleBtn('EMERGENCY STOP', true, '{stop_red}', '{stop_border}') && ok;
      if (!ok) setTimeout(apply, 100);
    }};
    apply();
    if (!window._dashBtnInterval) {{
      window._dashBtnInterval = setInterval(apply, 500);
    }}
    </script>
    """
    components.html(js, height=0)


def _disabled_radio_option(label: str, container=None) -> None:
    target = container or st
    target.markdown(
        f"""
        <div style="opacity:0.45; display:flex; align-items:center; margin-left:0.4rem;">
            <input type="radio" disabled style="margin-right:0.35rem;">
            <span>{label}</span>
        </div>
        """,
        unsafe_allow_html=True,
    )


def main() -> None:
    st.set_page_config(layout="wide", page_title="Go2 Telemetry")
    st.title("Go2 Telemetry Dashboard")

    node = get_ros_node()
    # Ensure callbacks are processed even if the background executor stalls.
    try:
        rclpy.spin_once(node, timeout_sec=0.0)
    except Exception:
        pass
    snapshot = node.snapshot()

    st.sidebar.header("Settings")
    status_map = snapshot["status"]
    mujoco_running = _get_fresh_status(
        status_map,
        "mujoco",
        time.monotonic(),
        float(snapshot["status_timeout_s"]),
    ) is True
    loco_ctrl_status = _get_fresh_status(
        status_map,
        "loco_ctrl",
        time.monotonic(),
        float(snapshot["status_timeout_s"]),
    )
    mpc_status = _get_fresh_status(
        status_map,
        "mpc",
        time.monotonic(),
        float(snapshot["status_timeout_s"]),
    )
    locomotion_active = (loco_ctrl_status is False) or (mpc_status is False) or (loco_ctrl_status is True and mpc_status is True)
    col_start, col_stop = st.sidebar.columns(2)
    if col_start.button("Start MuJoCo", key="start_mujoco", disabled=mujoco_running):
        client = st.session_state.get("svc_start")
        if client is None or not client.service_is_ready():
            st.sidebar.warning("Start service not available")
        else:
            client.call_async(Trigger.Request())
            st.sidebar.info("Start request sent")
    if col_stop.button("Stop MuJoCo", key="stop_mujoco", disabled=not mujoco_running):
        client = st.session_state.get("svc_stop")
        if client is None or not client.service_is_ready():
            st.sidebar.warning("Stop service not available")
        else:
            client.call_async(Trigger.Request())
            st.sidebar.info("Stop request sent")
    col_ctrl_start, col_ctrl_stop = st.sidebar.columns(2)
    if col_ctrl_start.button("Start Control Stack", key="start_ctrl", disabled=locomotion_active):
        client = st.session_state.get("svc_ctrl_start")
        if client is None or not client.service_is_ready():
            st.sidebar.warning("Control Stack start service not available")
        else:
            client.call_async(Trigger.Request())
            st.sidebar.info("Control Stack start request sent")
    if col_ctrl_stop.button("Stop Control Stack", key="stop_ctrl", disabled=not locomotion_active):
        client = st.session_state.get("svc_ctrl_stop")
        if client is None or not client.service_is_ready():
            st.sidebar.warning("Control Stack stop service not available")
        else:
            client.call_async(Trigger.Request())
            st.sidebar.info("Control Stack stop request sent")
    if st.sidebar.button("EMERGENCY STOP", key="emergency_stop", use_container_width=True):
        client = st.session_state.get("svc_emergency_stop")
        if client is None or not client.service_is_ready():
            st.sidebar.warning("Emergency stop service not available")
        else:
            client.call_async(Trigger.Request())
            st.sidebar.error("Emergency stop requested")
    _style_sidebar_buttons(mujoco_running, locomotion_active)

    st.sidebar.subheader("Plots")
    window_sec = 10.0
    refresh_ms = 50
    plot_refresh_s = 0.5
    use_autorefresh = hasattr(st, "autorefresh")
    if use_autorefresh:
        st.autorefresh(interval=refresh_ms, key="autorefresh")
    qdq_ok = _get_fresh_status(status_map, "qdq", time.monotonic(), float(snapshot["status_timeout_s"])) is True
    qdq_est_ok = _get_fresh_status(status_map, "qdq_est", time.monotonic(), float(snapshot["status_timeout_s"])) is True
    disable_pos = not (qdq_ok or qdq_est_ok)
    if disable_pos:
        st.session_state["show_pos"] = False
    show_pos = st.sidebar.checkbox("Show Position/Velocity", value=False, key="show_pos", disabled=disable_pos)
    data_mode = "True State (Simulator)"
    if show_pos:
        radio_key = "state_source"
        if not qdq_est_ok:
            st.session_state[radio_key] = "True State (Simulator)"
        if qdq_est_ok:
            data_mode = st.sidebar.radio(
                "State Source",
                options=["True State (Simulator)", "Estimator (InEKF)"],
                index=0,
                horizontal=False,
                key=radio_key,
            )
        else:
            data_mode = st.sidebar.radio(
                "State Source",
                options=["True State (Simulator)"],
                index=0,
                horizontal=False,
                key=radio_key,
            )
            _disabled_radio_option("Estimator (InEKF)", st.sidebar)
        if not qdq_est_ok:
            data_mode = "True State (Simulator)"
    show_rpy = st.sidebar.checkbox("Show RPY", value=False)
    show_cmd = st.sidebar.checkbox("Show Cmd", value=False)
    show_grf = st.sidebar.checkbox("Show GRF", value=False)
    max_points = int(snapshot.get("max_plot_points", 1000))

    st.subheader("Modules")
    render_status(snapshot, snapshot["status_timeout_s"])
    st.subheader("Topics")
    topic_available = snapshot.get("topic_available", {})
    topic_rates = snapshot.get("topic_rate", {})
    topic_names = snapshot.get("topic_names", {})
    topic_status_timeout = float(snapshot["status_timeout_s"])
    now = time.monotonic()

    def _topic_ok(topic_key: str, status_key: str) -> bool:
        return _get_fresh_status(snapshot["status"], status_key, now, topic_status_timeout) is True

    topic_rows = [
        ("lowstate", "lowstate"),
        ("mpc_forces", "mpc_forces"),
        ("joy", "joy"),
        ("qdq", "qdq"),
        ("qdq_est", "qdq_est"),
    ]
    badges = []
    for key, status_key in topic_rows:
        topic_key = topic_names.get(key, f"/{key}")
        ok = _topic_ok(topic_key, status_key)
        cls = "topic-ok" if ok else "topic-bad"
        badges.append(f"<span class=\"{cls}\">{topic_key}</span>")
    st.markdown(
        """
        <style>
        .topic-wrap { display:flex; flex-wrap:wrap; gap:0.4rem; }
        .topic-ok { background:#2ecc71; color:#fff; padding:0.2rem 0.5rem; border-radius:0.35rem;
                    font-weight:600; font-size:0.9rem; }
        .topic-bad { background:#e74c3c; color:#fff; padding:0.2rem 0.5rem; border-radius:0.35rem;
                     font-weight:600; font-size:0.9rem; }
        </style>
        """,
        unsafe_allow_html=True,
    )
    st.markdown(f"<div class=\"topic-wrap\">{''.join(badges)}</div>", unsafe_allow_html=True)
    st.subheader("Plots")

    cache_version = st.session_state.get("plot_cache_version", 0)
    if cache_version != 5:
        st.session_state["plot_cache"] = {}
        st.session_state["plot_cache_version"] = 5
    plot_cache = st.session_state.setdefault("plot_cache", {})
    now = time.monotonic()
    last_plot_update = float(st.session_state.get("last_plot_update", 0.0))
    update_plots = (now - last_plot_update) >= plot_refresh_s or not plot_cache
    if update_plots:
        st.session_state["last_plot_update"] = now

    tab_labels = []
    tab_kinds = []
    if show_pos:
        tab_labels.append("Position/Velocity")
        tab_kinds.append("pos")
    if show_rpy:
        tab_labels.append("RPY")
        tab_kinds.append("rpy")
    if show_cmd:
        tab_labels.append("Cmd")
        tab_kinds.append("cmd")
    if show_grf:
        tab_labels.append("GRF")
        tab_kinds.append("grf")

    if not tab_labels:
        st.info("All plots are disabled. Enable a plot tab in the sidebar to render charts.")
    else:
        tabs = st.tabs(tab_labels)
        for idx, kind in enumerate(tab_kinds):
            with tabs[idx]:
                if kind == "pos":
                    t_real, real = filter_window(snapshot["t_q"], snapshot["q_xyz"], window_sec)
                    t_est, est = filter_window(snapshot["t_q_est"], snapshot["q_xyz_est"], window_sec)
                    if data_mode == "True State (Simulator)":
                        t_plot, series_plot = decimate([float(x) for x in t_real], real, max_points) if t_real else ([], [])
                        labels = ["q_x", "q_y", "q_z"]
                        cache_key = "pos_position_true"
                    else:
                        t_plot, series_plot = decimate([float(x) for x in t_est], est, max_points) if t_est else ([], [])
                        labels = ["qdq_est_x", "qdq_est_y", "qdq_est_z"]
                        cache_key = "pos_position_est"

                    if update_plots or cache_key not in plot_cache:
                        if not t_plot or not series_plot:
                            plot_cache.pop(cache_key, None)
                        else:
                            series_plot = [[float(v) for v in s] for s in series_plot]
                            plot_cache[cache_key] = {labels[i]: series_plot[i] for i in range(len(labels))}

                    data = plot_cache.get(cache_key)
                    if data is None:
                        st.write("Position (m): no data")
                    else:
                        try:
                            clean = {k: [float(x) for x in v] for k, v in data.items()}
                        except Exception:
                            plot_cache.pop(cache_key, None)
                            st.write("Position (m): no data")
                            continue
                        st.markdown("**Position (m)**")
                        # Streamlit line_chart has issues with some list inputs in this environment.
                        # Use the cached matplotlib path to keep it stable.
                        fig, ax = plt.subplots()
                        for label, series in clean.items():
                            ax.plot(series, label=label)
                        ax.set_title("Position (m)")
                        ax.set_xlabel("sample")
                        ax.legend(loc="upper right")
                        st.pyplot(fig, clear_figure=True)
                elif kind == "rpy":
                    t, series = filter_window(snapshot["t_q"], snapshot["rpy"], window_sec)
                    if t:
                        roll_deg = np.array(series[0]) * 180.0 / np.pi
                        pitch_deg = np.array(series[1]) * 180.0 / np.pi
                        yaw_deg = np.array(series[2]) * 180.0 / np.pi
                        _render_cached_plot(
                            "rpy_roll_pitch",
                            lambda: _make_series_fig(
                                t,
                                [list(roll_deg), list(pitch_deg)],
                                ["roll", "pitch"],
                                title="Roll/Pitch (deg)",
                                ylim=(-5.0, 5.0),
                                max_points=max_points,
                            ),
                            plot_cache,
                            update_plots,
                            "Roll/Pitch (deg): no data",
                        )
                        _render_cached_plot(
                            "rpy_yaw",
                            lambda: _make_series_fig(
                                t,
                                [list(yaw_deg)],
                                ["yaw"],
                                title="Yaw (deg)",
                                ylim=(-180.0, 180.0),
                                max_points=max_points,
                            ),
                            plot_cache,
                            update_plots,
                            "Yaw (deg): no data",
                        )
                    else:
                        plot_cache.pop("rpy_roll_pitch", None)
                        plot_cache.pop("rpy_yaw", None)
                        st.write("RPY: no data")
                elif kind == "cmd":
                    t, series = filter_window(snapshot["t_cmd"], snapshot["cmd"], window_sec)
                    _render_cached_plot(
                        "cmd_locomotion",
                        lambda: _make_series_fig(
                            t,
                            series,
                            ["x_vel", "y_vel", "z_pos", "yaw_rate", "gait_hz"],
                            title="Locomotion Cmd",
                            max_points=max_points,
                        ),
                        plot_cache,
                        update_plots,
                        "Locomotion Cmd: no data",
                    )
                elif kind == "grf":
                    t, series = filter_window(snapshot["t_grf"], snapshot["grf_z"], window_sec)
                    _render_cached_plot(
                        "grf_fz",
                        lambda: _make_series_fig(
                            t,
                            series,
                            ["FL", "FR", "RL", "RR"],
                            title="GRF Fz (N)",
                            max_points=max_points,
                        ),
                        plot_cache,
                        update_plots,
                        "GRF Fz (N): no data",
                    )

    if not use_autorefresh:
        _autorefresh_fallback(refresh_ms)


if __name__ == "__main__":
    main()
