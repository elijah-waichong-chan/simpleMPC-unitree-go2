import rclpy
from rclpy.node import Node
from pathlib import Path
import mujoco as mj
import time
import numpy as np

from unitree_go.msg import LowState, LowCmd

 

class MuJoCoRobot(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("mujoco_robot_node")

        self.sim_hz = 1000.0
        self.pub_hz = 10.0

        repo = Path(__file__).resolve().parents[4]
        xml_path = repo / "models" / "MJCF" / "go2" / "scene.xml"

        # load MuJoCo
        self.model = mj.MjModel.from_xml_path(str(xml_path))
        self.data = mj.MjData(self.model)

        self.imu_quat_sl = slice(36, 40)
        self.imu_gyro_sl = slice(40, 43)
        self.imu_acc_sl  = slice(43, 46)


        # set initial state
        self.base_pos = np.array([0.0, 0.0, 0.27])
        self.base_quad = np.array([0.0, 0.0, 0.0, 1.0])
        self.FL_joint_angle = np.array([0.0, 0.9, -1.8])
        self.FR_joint_angle =  np.array([0.0, 0.9, -1.8])
        self.RL_joint_angle = np.array([0.0, 0.9, -1.8])
        self.RR_joint_angle = np.array([0.0, 0.9, -1.8])

        self.data.qpos[:] = [self.base_pos[0], self.base_pos[1], self.base_pos[2],
                             self.base_quad[3], self.base_quad[0], self.base_quad[1], self.base_quad[2],
                             self.FL_joint_angle[0], self.FL_joint_angle[1], self.FL_joint_angle[2],
                             self.FR_joint_angle[0], self.FR_joint_angle[1], self.FR_joint_angle[2],
                             self.RL_joint_angle[0], self.RL_joint_angle[1], self.RL_joint_angle[2],
                             self.RR_joint_angle[0], self.RR_joint_angle[1], self.RR_joint_angle[2]]
        mj.mj_forward(self.model, self.data)

        # ROS pub
        self.pub_lowstate = self.create_publisher(LowState, "/lowstate", 10)

        # timers
        self.sim_dt = 1.0 / self.sim_hz
        self.pub_dt = 1.0 / self.pub_hz
        self.sim_timer = self.create_timer(self.sim_dt, self.sim_step)
        self.pub_timer = self.create_timer(self.pub_dt, self.publish_lowstate)

        self._t0 = time.time()
        
        self.get_logger().info(f"Loaded MuJoCo XML: {xml_path}")
        self.get_logger().info(f"sim_hz={self.sim_hz}, pub_hz={self.pub_hz}")

    def sim_step(self):
        mj.mj_step(self.model, self.data)

    def publish_lowstate(self):
        msg = LowState()

        # test
        for k in range(4):
            msg.foot_force[k] = 100
            msg.foot_force_est[k] = 100

        # publish joint positions
        q = self.data.qpos

        for i in range(12):
            msg.motor_state[i].q = float(q[6 + i])

        # Read MuJoCo sensors
        quat_wxyz = self.data.sensordata[self.imu_quat_sl]  # [w,x,y,z]
        gyro_xyz  = self.data.sensordata[self.imu_gyro_sl]  # [wx,wy,wz] in imu(site) frame
        acc_xyz   = self.data.sensordata[self.imu_acc_sl]   # [ax,ay,az] in imu(site) frame

        # publish IMU data
        msg.imu_state.quaternion[0] = float(quat_wxyz[0])  # w
        msg.imu_state.quaternion[1] = float(quat_wxyz[1])  # x
        msg.imu_state.quaternion[2] = float(quat_wxyz[2])  # y
        msg.imu_state.quaternion[3] = float(quat_wxyz[3])  # z

        msg.imu_state.gyroscope[0] = float(gyro_xyz[0])
        msg.imu_state.gyroscope[1] = float(gyro_xyz[1])
        msg.imu_state.gyroscope[2] = float(gyro_xyz[2])

        msg.imu_state.accelerometer[0] = float(acc_xyz[0])
        msg.imu_state.accelerometer[1] = float(acc_xyz[1])
        msg.imu_state.accelerometer[2] = float(acc_xyz[2])

        self.pub_lowstate.publish(msg)
        self.get_logger().info(f"Published IMU data: accel_z ={msg.imu_state.accelerometer[2]}")



def main():
    rclpy.init()
    node = MuJoCoRobot()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
