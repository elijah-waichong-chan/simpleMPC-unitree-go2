from dataclasses import dataclass
import numpy as np


@dataclass
class ConfigurationState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.27
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    theta1: float = 0.0
    theta2: float = 0.9
    theta3: float = -1.8
    theta4: float = 0.0
    theta5: float = 0.9
    theta6: float = -1.8
    theta7: float = 0.0
    theta8: float = 0.9
    theta9: float = -1.8
    theta10: float = 0.0
    theta11: float = 0.9
    theta12: float = -1.8

    
    def compute_q(self):
        q = np.array([self.x, self.y, self.z,                      
                    self.qx, self.qy, self.qz, self.qw,                 
                    self.theta1, self.theta2, self.theta3,
                    self.theta4, self.theta5, self.theta6,
                    self.theta7, self.theta8, self.theta9,
                    self.theta10, self.theta11, self.theta12], dtype=float) 
        return q

    def update_config(self, q):
        self.x = q[0]
        self.y = q[1]
        self.z = q[2]
        self.qx = q[3]
        self.qy = q[4]
        self.qz = q[5]
        self.qw = q[6]
        self.theta1 = q[7]
        self.theta2 = q[8]
        self.theta3 = q[9]
        self.theta4 = q[10]
        self.theta5 = q[11]
        self.theta6 = q[12]
        self.theta7 = q[13]
        self.theta8 = q[14]
        self.theta9 = q[15]
        self.theta10 = q[16]
        self.theta11 = q[17]
        self.theta12 = q[18]

     
@dataclass
class RigidBodyState:
    x_pos: float = 0.0
    y_pos: float = 0.0
    z_pos: float = 0.0
    x_vel: float = 0.0
    y_vel: float = 0.0
    z_vel: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0

@dataclass
class RigidBodyTraj:
    time: np.ndarray = np.empty(0)
    x_pos_ref: np.ndarray = np.empty(0)
    y_pos_ref: np.ndarray = np.empty(0)
    z_pos_ref: np.ndarray = np.empty(0)
    x_vel_ref: np.ndarray = np.empty(0)
    y_vel_ref: np.ndarray = np.empty(0)
    z_vel_ref: np.ndarray = np.empty(0)
    roll_ref: np.ndarray = np.empty(0)
    pitch_ref: np.ndarray = np.empty(0)
    yaw_ref: np.ndarray = np.empty(0)
    roll_rate_ref: np.ndarray = np.empty(0)
    pitch_rate_ref: np.ndarray = np.empty(0)
    yaw_rate_ref: np.ndarray = np.empty(0)
    fl_foot_placement: np.ndarray = np.empty(0)
    fr_foot_placement: np.ndarray = np.empty(0)
    rl_foot_placement: np.ndarray = np.empty(0)
    rr_foot_placement: np.ndarray = np.empty(0)
    fl_foot_placement_vec: np.ndarray = np.empty(0)
    fr_foot_placement_vec: np.ndarray = np.empty(0)
    rl_foot_placement_vec: np.ndarray = np.empty(0)
    rr_foot_placement_vec: np.ndarray = np.empty(0)