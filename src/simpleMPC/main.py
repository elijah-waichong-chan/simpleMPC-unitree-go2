import numpy as np
from robot_classes import RigidBodyState
from robot_classes import RigidBodyTraj
from trajectoryPlanner import generateConstantTraj
from centroidalDynamics import continuousDynamics
from centroidalDynamics import discreteDynamics
import matplotlib.pyplot as plt

state = RigidBodyState()
traj = RigidBodyTraj()

traj = generateConstantTraj(state, 0, 0.5, 0, 0, 0.1, 5, 0.6, 0.7)

# plt.figure(figsize=(8, 6))
# plt.plot(traj.x_pos_ref, traj.y_pos_ref)
# plt.plot(traj.fl_foot_placement[0,:], traj.fl_foot_placement[1,:], 'o')
# plt.plot(traj.fr_foot_placement[0,:], traj.fr_foot_placement[1,:], 'o')
# plt.plot(traj.rl_foot_placement[0,:], traj.rl_foot_placement[1,:], 'o')
# plt.plot(traj.rr_foot_placement[0,:], traj.rr_foot_placement[1,:], 'o')
# plt.show()

[A_continuous, B_continuous] = continuousDynamics(state, traj)

[Ad, Bd] = discreteDynamics(A_continuous, B_continuous, 0.02)