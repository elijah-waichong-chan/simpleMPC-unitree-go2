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
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    yaw_rate: float = 0.0


class RigidBodyTraj:
    time: np.ndarray = np.empty(0)
    x_pos_ref: np.ndarray = np.empty(0)
    y_pos_ref: np.ndarray = np.empty(0)
    z_pos_ref: np.ndarray = np.empty(0)
    x_vel_ref: np.ndarray = np.empty(0)
    y_vel_ref: np.ndarray = np.empty(0)
    z_vel_ref: np.ndarray = np.empty(0)
    roll_rate_ref: np.ndarray = np.empty(0)
    pitch_rate_ref: np.ndarray = np.empty(0)
    yaw_rate_ref: np.ndarray = np.empty(0)