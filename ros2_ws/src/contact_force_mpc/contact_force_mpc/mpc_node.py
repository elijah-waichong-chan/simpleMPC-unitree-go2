import rclpy
from rclpy.node import Node
import time
import numpy as np
import pinocchio as pin

from contact_force_mpc.go2_robot_data import PinGo2Model
from contact_force_mpc.com_trajectory import ComTraj
from contact_force_mpc.centroidal_mpc import CentroidalMPC
from contact_force_mpc.gait import Gait

from go2_msgs.msg import QDq, MpcForces
from std_msgs.msg import Float64 # type: ignore
from rclpy.qos import qos_profile_sensor_data # type: ignore


from threading import Lock


class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_node")

        # Parameters
        self.gait_hz = 3.0
        self.gait_duty = 0.6
        self.gait_t = 1.0 / self.gait_hz

        self.mpc_dt = self.gait_t / 16
        self.mpc_hz = 1.0 / self.mpc_dt

        # Trajectory Reference Setting (defaults)
        self.x_vel_des_body = 0.0
        self.y_vel_des_body = 0.0
        self.z_pos_des_body = 0.27
        self.yaw_rate_des_body = 0.0

        # State
        self.sim_time_t0 = None
        self.sim_time_now = 0.0
        self._have_state = False
        self.last_update_time = None

        # Models
        self.go2_mpc = PinGo2Model()
        self.traj = ComTraj(self.go2_mpc)
        self.gait = Gait(self.gait_hz, self.gait_duty)
        self.traj.generate_traj(
            self.go2_mpc,
            self.gait,
            0.0,
            self.x_vel_des_body,
            self.y_vel_des_body,
            self.z_pos_des_body,
            self.yaw_rate_des_body,
            time_step=self.mpc_dt,
        )
        self.mpc = CentroidalMPC(self.go2_mpc, self.traj)

        # ROS pub/sub
        self.pub_mpc = self.create_publisher(MpcForces, "/mpc_forces", qos_profile_sensor_data)
        self.sub_robotstate = self.create_subscription(QDq, "/qdq", self.store_state, qos_profile_sensor_data)
        self.mpc_loop_ms = self.create_publisher(Float64, "/timing/mpc_loop_ms", 10)

        # Timer
        self.mpc_timer = self.create_timer(self.mpc_dt, self.mpc_step)

        # Locks
        self._go2_lock = Lock()

        self.get_logger().info("MPC node is running...")

    def _pub_ms(self, pub, ms: float):
        m = Float64()
        m.data = float(ms)
        pub.publish(m)

    def store_state(self, msg: QDq):
        if self.sim_time_t0 is None:
            self.sim_time_t0 = msg.sim_time
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

            self.go2_mpc.update_model(q_pin, dq_pin)

    def mpc_step(self):
        t0 = time.perf_counter()
        if not self._have_state:
            return

        now = self.get_clock().now()
        if self.last_update_time is None or (now - self.last_update_time).nanoseconds * 1e-9 > 1.0:
            self._have_state = False
            return

        time_now_s = float(self.sim_time_now - self.sim_time_t0)

        with self._go2_lock:
            self.traj.generate_traj(
                self.go2_mpc,
                self.gait,
                time_now_s,
                self.x_vel_des_body,
                self.y_vel_des_body,
                self.z_pos_des_body,
                self.yaw_rate_des_body,
                time_step=self.mpc_dt,
            )

            sol = self.mpc.solve_QP(self.go2_mpc, self.traj, False)

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
