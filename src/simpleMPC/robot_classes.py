from dataclasses import dataclass, field
import numpy as np


@dataclass
class ConfigurationState:
    x_pos: float = 0.0
    y_pos: float = 0.0
    z_pos: float = 0.3

    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0

    theta1, theta2, theta3 = 1, 0, 0        # leg 1 joint angle (rad)
    theta4, theta5, theta6 = 0, 0, 0        # leg 2 joint angle (rad)
    theta7, theta8, theta9 = 0, 0, 0        # leg 3 joint angle (rad)
    theta10, theta11, theta12 = 0, 0, 0     # leg 4 joint angle (rad)

    q: np.ndarray = field(default_factory=lambda: np.array(
        [1.0, 0.0, 0.0,   # leg 1
         0.0, 0.0, 0.0,   # leg 2
         0.0, 0.0, 0.0,   # leg 3
         0.0, 0.0, 0.0],  # leg 4
        dtype=float
    ))

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