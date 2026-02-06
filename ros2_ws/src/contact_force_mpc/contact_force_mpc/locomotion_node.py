import rclpy
from rclpy.node import Node
import time
import numpy as np
import pinocchio as pin
import math

from contact_force_mpc.go2_robot_data import PinGo2Model
from contact_force_mpc.leg_controller import LegController
from contact_force_mpc.gait import Gait

from unitree_go.msg import LowState, LowCmd
from go2_msgs.msg import QDq, MpcForces
from std_msgs.msg import Float64

from threading import Lock
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data


class LocomotionMPC(Node):
    def __init__(self):
        super().__init__("contact_force_mpc_node")
        self.gait_hz = 3.0
        self.gait_duty = 0.6
        self.gait_t = 1.0 / self.gait_hz

        self.ctrl_hz = 250.0
        self.ctrl_dt = 1.0 / self.ctrl_hz

        # Motor Limits
        self.hip_lim = 23.7
        self.abd_lim = 23.7
        self.knee_lim = 45.43
        self.safety = 0.9

        self.tau_lim = self.safety * np.array([
            self.hip_lim, self.abd_lim, self.knee_lim,   # FL: hip, thigh, calf
            self.hip_lim, self.abd_lim, self.knee_lim,   # FR
            self.hip_lim, self.abd_lim, self.knee_lim,   # RL
            self.hip_lim, self.abd_lim, self.knee_lim,   # RR
        ])

        # Leg indexing
        self.leg_slice = {
            "FL": slice(0, 3),
            "FR": slice(3, 6),          
            "RL": slice(6, 9),
            "RR": slice(9, 12),
        }

        # Trajectory Reference Setting (defaults)
        self.x_vel_des_body = 0.0
        self.y_vel_des_body = 0.0
        self.z_pos_des_body = 0.27
        self.yaw_rate_des_body = 0.0

        # Storage Variables
        self.mpc_force_world = np.zeros(12, dtype=float)
        self.tau_raw = np.zeros(12, dtype=float)
        self.q_cmd = np.zeros(12, dtype=float)
        self.dq_cmd = np.zeros(12, dtype=float)
        self.tau_hold = np.zeros(12, dtype=float)

        # Initialization
        self.go2 = PinGo2Model()
        self.leg_controller = LegController()
        self.gait = Gait(self.gait_hz, self.gait_duty)

        self._pin_updated = False
        self._have_state = False

        # ROS pub/sub
        self.pub_lowcmd = self.create_publisher(LowCmd, "/lowcmd", qos_profile_sensor_data)
        self.cb_group_state = MutuallyExclusiveCallbackGroup()
        self.cb_group_mpc = MutuallyExclusiveCallbackGroup()
        self.cb_group_ctrl_loop = MutuallyExclusiveCallbackGroup()

        self.sub_robotstate = self.create_subscription(
            QDq, "/qdq", self.store_state, 
            qos_profile_sensor_data, 
            callback_group=self.cb_group_state
        )

        self.sub_mpc = self.create_subscription(
            MpcForces, "/mpc_forces", self.store_mpc_forces, 
            qos_profile_sensor_data, 
            callback_group=self.cb_group_mpc
        )
        # Timing publishers
        self.storing_ms  = self.create_publisher(Float64, "/timing/storing_state_ms", 10)
        self.ctrl_loop_ms  = self.create_publisher(Float64, "/timing/ctrl_loop_ms", 10)

        # Control loop runs on a fixed-rate timer.
        self.ctrl_timer = self.create_timer(
            self.ctrl_dt, self.ctrl_on_state, callback_group=self.cb_group_ctrl_loop
        )
        self._last_wait_log_t = self.get_clock().now()
        self.last_update_time = None
        self._last_ctrl_t = None
        self._last_ctrl_warn_t = self.get_clock().now()
        self._force_lock = Lock()
        self._go2_lock = Lock()
        self._have_mpc = False
        self.last_mpc_stamp = None
        self._mpc_timeout_s = 0.5
        self._mpc_force_eps = 1e-6
        self._loop_log_t0 = self.get_clock().now()
        self._loop_acc_s = 0.0
        self._loop_acc_n = 0
        self._loop_max_s = 0.0
        self._seg_max = {"targets": 0.0, "compute": 0.0, "clip": 0.0, "other": 0.0}
        self._lowcmd_count = 0

        # --- Change gait duty after 5 seconds (one-shot) ---
        self._gait_duty_switched = False
        # self._switch_timer = self.create_timer(5.0, self._switch_gait_duty_once)
        # self._height_timer = self.create_timer(0.1, self._sine_height)

        self.ctrl_clock_t0 = None

        self.get_logger().info("Locomotion controller is running...")

    def _pub_ms(self, pub, ms: float):
        m = Float64()
        m.data = float(ms)
        pub.publish(m)

    def _switch_gait_duty_once(self):
        if self._gait_duty_switched:
            return

        # Switch desired duty
        self.gait_duty = 0.0

        # Recreate gait object so downstream uses the new duty.
        self.gait = Gait(self.gait_hz, self.gait_duty)

        self.get_logger().info("Switched gait_duty to 0.6 after 5 seconds.")

        self._gait_duty_switched = True

        # Cancel timer so it doesn't keep firing
        self._switch_timer.cancel()

    def _sine_height(self):
        self.z_pos_des_body = 0.25 + 0.05 * math.sin(2 * math.pi * 0.5 * self.sim_time_now)

    def ctrl_on_state(self):
        now_clock = self.get_clock().now()
        if self._last_ctrl_t is not None:
            dt = (now_clock - self._last_ctrl_t).nanoseconds * 1e-9
            if dt > 1.5 * self.ctrl_dt:
                if (now_clock - self._last_ctrl_warn_t).nanoseconds > 2e9:
                    # self.get_logger().warn(
                    #     f"CTRL loop behind: dt={dt:.4f}s (target {self.ctrl_dt:.4f}s)"
                    # )
                    self._last_ctrl_warn_t = now_clock
        self._last_ctrl_t = now_clock

        t0 = time.perf_counter()
        if not self._have_state:
            self.tau_hold[:] = 0.0
            return
        
        time_now_s = float(self.sim_time_now - self.ctrl_clock_t0)

        with self._force_lock:
            t_seg = time.perf_counter()
            # Update desired world targets used by swing controller
            vel_des_world = self.go2.R_z @ np.array(
                [self.x_vel_des_body, self.y_vel_des_body, 0.0], dtype=float
            )
            self.go2.x_vel_des_world = float(vel_des_world[0])
            self.go2.y_vel_des_world = float(vel_des_world[1])
            self.go2.yaw_rate_des_world = float(self.yaw_rate_des_body)
            # Keep position target near current COM (prevents large jumps)
            if not hasattr(self, "_pos_des_world"):
                self._pos_des_world = self.go2.pos_com_world.copy()
            max_pos_error = 0.1
            x0, y0 = self.go2.pos_com_world[0], self.go2.pos_com_world[1]
            self._pos_des_world[0] = np.clip(
                self._pos_des_world[0], x0 - max_pos_error, x0 + max_pos_error
            )
            self._pos_des_world[1] = np.clip(
                self._pos_des_world[1], y0 - max_pos_error, y0 + max_pos_error
            )
            self._pos_des_world[2] = self.z_pos_des_body
            self.go2.x_pos_des_world = float(self._pos_des_world[0])
            self.go2.y_pos_des_world = float(self._pos_des_world[1])
            t_targets = time.perf_counter() - t_seg

            # Compute joint torques
            t_seg = time.perf_counter()
            self.tau_raw = self.leg_controller.compute_all_leg_torques(self.go2, self.gait, self.mpc_force_world, time_now_s)
            t_compute = time.perf_counter() - t_seg

        # Saturate + hold
        t_seg = time.perf_counter()
        self.tau_hold = np.clip(self.tau_raw, -self.tau_lim, self.tau_lim)
        t_clip = time.perf_counter() - t_seg
        t1 = time.perf_counter()
        self._pub_ms(self.ctrl_loop_ms, (t1 - t0) * 1000.0)  # Placeholder for timing

        # Log average loop time ~1 Hz
        self._loop_acc_s += (t1 - t0)
        self._loop_acc_n += 1
        loop_s = (t1 - t0)
        if loop_s > self._loop_max_s:
            self._loop_max_s = loop_s
        other_s = max(loop_s - (t_targets + t_compute + t_clip), 0.0)
        self._seg_max["targets"] = max(self._seg_max["targets"], t_targets)
        self._seg_max["compute"] = max(self._seg_max["compute"], t_compute)
        self._seg_max["clip"] = max(self._seg_max["clip"], t_clip)
        self._seg_max["other"] = max(self._seg_max["other"], other_s)
        now_clock = self.get_clock().now()
        dt_s = (now_clock - self._loop_log_t0).nanoseconds * 1e-9
        if dt_s >= 1.0:
            loop_ms = (self._loop_acc_s / self._loop_acc_n) * 1000.0 if self._loop_acc_n > 0 else 0.0
            lowcmd_hz = (self._lowcmd_count / dt_s) if dt_s > 0.0 else 0.0
            self.get_logger().info(
                "ctrl_loop_ms=%.3f  ctrl_loop_max_ms=%.3f  seg_max_ms: "
                "targets=%.3f compute=%.3f clip=%.3f other=%.3f  lowcmd_hz=%.1f"
                % (
                    loop_ms,
                    self._loop_max_s * 1000.0,
                    self._seg_max["targets"] * 1000.0,
                    self._seg_max["compute"] * 1000.0,
                    self._seg_max["clip"] * 1000.0,
                    self._seg_max["other"] * 1000.0,
                    lowcmd_hz,
                )
            )
            self._loop_log_t0 = now_clock
            self._loop_acc_s = 0.0
            self._loop_acc_n = 0
            self._loop_max_s = 0.0
            self._seg_max = {"targets": 0.0, "compute": 0.0, "clip": 0.0, "other": 0.0}
            self._lowcmd_count = 0

        # Publish only if MPC is alive or forces are non-zero
        mpc_alive = (
            self._have_mpc
            and self.last_mpc_stamp is not None
            and (now_clock - self.last_mpc_stamp).nanoseconds * 1e-9 <= self._mpc_timeout_s
        )
        with self._force_lock:
            forces_nonzero = np.any(np.abs(self.mpc_force_world) > self._mpc_force_eps)
        if mpc_alive or forces_nonzero:
            self.publish_lowcmd()

    def store_mpc_forces(self, msg: MpcForces):
        forces = np.array(msg.forces, dtype=float)
        if forces.size >= 12:
            forces = forces[:12]
        else:
            forces = np.pad(forces, (0, 12 - forces.size), mode="constant")
        with self._force_lock:
            self.mpc_force_world[:] = forces
        self._have_mpc = True
        self.last_mpc_stamp = self.get_clock().now()

    def publish_lowcmd(self):
        msg = LowCmd()
        mujoco_to_unitree = [3,4,5, 0,1,2, 9,10,11, 6,7,8]
        for i in range(12):
            msg.motor_cmd[i].tau = float(self.tau_hold[mujoco_to_unitree[i]])
            msg.motor_cmd[i].q = 0.0
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].kp = 0.0
            msg.motor_cmd[i].kd = 0.0

        self.pub_lowcmd.publish(msg)
        self._lowcmd_count += 1

    def store_state(self, msg: QDq):
        if self.ctrl_clock_t0 is None:
            self.ctrl_clock_t0 = msg.sim_time
        self.sim_time_now = msg.sim_time

        self.q  = np.array(msg.q, dtype=float)
        self.dq = np.array(msg.dq, dtype=float)
        self.update_pin()
        self._have_state = True
        self.last_update_time = self.get_clock().now()
        self._pub_ms(self.storing_ms, 1.0)  # Placeholder for timing
        # Control runs on timer; no direct call here.

    def update_pin(self):
        with self._go2_lock:
            qw, qx, qy, qz = self.q[3:7]
            # pin.Quaternion uses qw qx qy qz
            R = pin.Quaternion(qw, qx, qy, qz).toRotationMatrix()  # body -> world
            v_world = self.dq[0:3]
            w_body = self.dq[3:6]
            v_body = R.T @ v_world

            # Convert to Pin
            # configuration uses qx qy qz qw
            q_pin  = np.concatenate([self.q[0:3], [qx, qy, qz, qw], self.q[7:]])
            dq_pin = np.concatenate([v_body, w_body, self.dq[6:]])

            self.go2.update_model(q_pin, dq_pin)
            self._pin_updated = True

def main():
    rclpy.init()
    node = LocomotionMPC()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
