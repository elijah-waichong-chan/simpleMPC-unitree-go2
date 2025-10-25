from dataclasses import dataclass
import numpy as np

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