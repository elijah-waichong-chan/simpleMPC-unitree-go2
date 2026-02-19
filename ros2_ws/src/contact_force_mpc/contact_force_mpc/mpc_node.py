import rclpy # type: ignore
from rclpy.node import Node # type: ignore
import time
import numpy as np
import pinocchio as pin # type: ignore

from contact_force_mpc.go2_robot_data import PinGo2Model
from contact_force_mpc.com_trajectory import ComTraj
from contact_force_mpc.centroidal_mpc import CentroidalMPC
from contact_force_mpc.gait import Gait

from go2_msgs.msg import QDq, MpcForces, LocomotionCmd
from std_msgs.msg import Float64, Bool # type: ignore
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy # type: ignore
from rclpy.qos import qos_profile_sensor_data # type: ignore


from threading import Lock


class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_node")

        # Parameters
        self.gait_hz = 3.0
        self.gait_duty = 1.0
        self.gait_t = 1.0 / self.gait_hz
        self.gait_hz_min = float(self.declare_parameter("gait_hz_min", 2.0).value)
        self.gait_hz_max = float(self.declare_parameter("gait_hz_max", 4.0).value)

        self.mpc_dt = self.gait_t / 16
        self.mpc_hz = 1.0 / self.mpc_dt

        # Trajectory Reference Setting (defaults)
        self.x_vel_des_body = 0.0
        self.y_vel_des_body = 0.0
        self.z_pos_des_body = 0.3
        self.yaw_rate_des_body = 0.0
        self.z_pos_min = float(self.declare_parameter("z_pos_min", 0.20).value)
        self.z_pos_max = float(self.declare_parameter("z_pos_max", 0.40).value)

        # State
        self.sim_time_now = 0.0
        self._have_state = False
        self.last_update_time = None

        # Models
        self.go2 = PinGo2Model()
        self.traj = ComTraj(self.go2)
        self.gait = Gait(self.gait_hz, self.gait_duty)
        self.traj.generate_traj(
            self.go2,
            self.gait,
            0.0,
            self.x_vel_des_body,
            self.y_vel_des_body,
            self.z_pos_des_body,
            self.yaw_rate_des_body,
            time_step=self.mpc_dt,
        )
        # Cost matrix Q (diag) as a parameter
        default_q = [1.0, 1.0, 50.0, 10.0, 20.0, 1.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0]
        cost_q = self.declare_parameter("cost_q", default_q).value
        try:
            cost_q = [float(x) for x in cost_q]
        except Exception:
            self.get_logger().warn("cost_q parameter is invalid; using default.")
            cost_q = default_q
        if len(cost_q) != len(default_q):
            self.get_logger().warn(
                f"cost_q length {len(cost_q)} != {len(default_q)}; using default.")
            cost_q = default_q
        self.cost_q_diag = cost_q
        self.debug_publish = bool(self.declare_parameter("debug_publish", True).value)
        self.time_offset = float(self.declare_parameter("gait_time_offset", 0.0).value)
        self.mpc = CentroidalMPC(self.go2, self.traj, q_diag=self.cost_q_diag)
        qdq_topic = str(self.declare_parameter("qdq_topic", "/qdq").value)
        locomotion_cmd_topic = str(
            self.declare_parameter("locomotion_cmd_topic", "/locomotion_cmd").value
        )

        # ROS pub/sub
        self.pub_mpc = self.create_publisher(MpcForces, "/mpc_forces", qos_profile_sensor_data)
        self.sub_robotstate = self.create_subscription(QDq, qdq_topic, self.store_state, qos_profile_sensor_data)
        self.sub_loco_cmd = self.create_subscription(
            LocomotionCmd, locomotion_cmd_topic, self.store_locomotion_cmd, qos_profile_sensor_data
        )
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._inekf_running = False
        self.sub_inekf_status = self.create_subscription(
            Bool, "/status/inekf/is_running", self._on_inekf_status, status_qos
        )
        self.mpc_loop_ms = self.create_publisher(Float64, "/timing/mpc_loop_ms", 10)
        self._is_running = False
        self.pub_status = self.create_publisher(Bool, "/status/mpc/is_running", status_qos)
        self.pub_status.publish(Bool(data=False))
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # Timer
        self.mpc_timer = self.create_timer(self.mpc_dt, self.mpc_step)

        # Locks
        self._go2_lock = Lock()

        self.get_logger().info("MPC node is running...")

    def store_locomotion_cmd(self, msg: LocomotionCmd):
        self.x_vel_des_body = float(msg.x_vel)
        self.y_vel_des_body = float(msg.y_vel)
        self.yaw_rate_des_body = float(msg.yaw_rate)
        self.z_pos_des_body = float(np.clip(msg.z_pos, self.z_pos_min, self.z_pos_max))
        next_hz = float(np.clip(msg.gait_hz, self.gait_hz_min, self.gait_hz_max))
        if next_hz != self.gait_hz:
            self.gait_hz = next_hz
            self.gait_t = 1.0 / self.gait_hz if self.gait_hz > 0.0 else 1.0
            self.gait.set_gait_hz(self.gait_hz)

    def _pub_ms(self, pub, ms: float):
        if not self.debug_publish:
            return
        m = Float64()
        m.data = float(ms)
        pub.publish(m)

    def _on_inekf_status(self, msg: Bool):
        self._inekf_running = bool(msg.data)

    def _publish_status(self):
        self.pub_status.publish(Bool(data=self._is_running))

    def store_state(self, msg: QDq):
        self.sim_time_now = msg.sim_time

        self.q = np.array(msg.q, dtype=float)
        self.dq = np.array(msg.dq, dtype=float)
        self.update_pin()
        self._have_state = True
        self.last_update_time = self.get_clock().now()

    def update_pin(self):
        with self._go2_lock:
            qw, qx, qy, qz = self.q[3:7]
            R = pin.Quaternion(qw, qx, qy, qz).toRotationMatrix()  # body -> world
            v_world = self.dq[0:3]
            w_body = self.dq[3:6]
            v_body = R.T @ v_world

            # Convert to Pin
            q_pin = np.concatenate([self.q[0:3], [qx, qy, qz, qw], self.q[7:]])
            dq_pin = np.concatenate([v_body, w_body, self.dq[6:]])

            self.go2.update_model(q_pin, dq_pin)

    def mpc_step(self):
        t0 = time.perf_counter()
        if not self._have_state or not self._inekf_running:
            self._is_running = False
            return

        now = self.get_clock().now()
        if self.last_update_time is None or (now - self.last_update_time).nanoseconds * 1e-9 > 1.0:
            self._have_state = False
            self._is_running = False
            return

        time_now_s = float(self.sim_time_now + self.time_offset)

        with self._go2_lock:
            self.traj.generate_traj(
                self.go2,
                self.gait,
                time_now_s,
                self.x_vel_des_body,
                self.y_vel_des_body,
                self.z_pos_des_body,
                self.yaw_rate_des_body,
                time_step=self.mpc_dt,
            )

            sol = self.mpc.solve_QP(self.go2, self.traj, False)

        N = self.traj.N
        w_opt = sol["x"].full().flatten()
        U_opt = w_opt[12 * (N) :].reshape((12, N), order="F")
        force = U_opt[:, 0]

        msg = MpcForces()
        # stamp based on sim time
        secs = int(self.sim_time_now)
        nsecs = int((self.sim_time_now - secs) * 1e9)
        msg.stamp.sec = secs
        msg.stamp.nanosec = nsecs
        msg.forces = force.tolist()
        msg.dt = float(self.mpc_dt)
        self.pub_mpc.publish(msg)
        self._is_running = True

        t1 = time.perf_counter()
        self._pub_ms(self.mpc_loop_ms, (t1 - t0) * 1000.0)


def main():
    rclpy.init()
    node = MPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
