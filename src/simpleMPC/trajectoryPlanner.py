import numpy as np
import matplotlib.pyplot as plt
from robot_classes import RigidBodyState
from robot_classes import RigidBodyTraj


def generateConstantTraj(state: RigidBodyState,
                 x_vel: float,
                 y_vel: float,
                 z_position: float,
                 yaw_rate: float,
                 time_step: float,
                 time_horizon: float) -> RigidBodyTraj:
    
    N = int(np.ceil(time_horizon / time_step)) # number of sequences to output
    t_vec = np.arange(N) * time_step # time vector

    trajectory = RigidBodyTraj()

    trajectory.time = t_vec
    trajectory.x_pos_ref = state.x_pos + x_vel * t_vec
    trajectory.x_vel_ref = np.full(N, x_vel, dtype=float)
    trajectory.y_pos_ref = state.y_pos + y_vel * t_vec
    trajectory.y_vel_ref = np.full(N, y_vel, dtype=float)
    trajectory.z_pos_ref = np.full(N, z_position, dtype=float)
    trajectory.z_vel_ref = np.full(N, 0, dtype=float)
    trajectory.yaw_rate_ref = np.full(N, yaw_rate, dtype=float)
    trajectory.pitch_rate_ref = np.full(N, 0, dtype=float)
    trajectory.roll_rate_ref = np.full(N, 0, dtype=float)

    return trajectory

# Test Code
state = RigidBodyState()
traj = generateConstantTraj(state, 0, 1, 0, 0, 0.1, 1)

plt.figure(figsize=(8, 6))
plt.plot(traj.x_pos_ref, traj.y_pos_ref)
plt.show()

