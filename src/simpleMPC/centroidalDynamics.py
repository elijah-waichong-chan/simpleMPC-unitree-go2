import pinocchio as pin 
import numpy as np
from numpy import sin, cos

from robot_classes import RigidBodyState
from robot_classes import RigidBodyTraj
from readURDF import createFloatingBaseModel
from trajectoryPlanner import generateConstantTraj

model, data, vmodel, vdata, cmodel, cdata  = createFloatingBaseModel()

q = pin.neutral(model)
v = np.zeros(model.nv)

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)
pin.ccrba(model, data, q, v) 

def skew(vector):
    if vector.shape != (3,):
        raise ValueError("Input vector must be a 3-element array.")

    return np.array([
        [0, -vector[2], vector[1]],
        [vector[2], 0, -vector[0]],
        [-vector[1], vector[0], 0]
    ])



def continuousDynamics(state: RigidBodyState,
                       traj: RigidBodyTraj):
    
    m = data.Ig.mass
    I_world = data.Ig.inertia
    I_inv = np.invert(I_world)
    skew_r1 = skew(traj.fl_foot_placement)
    skew_r2 = skew(traj.fr_foot_placement)
    skew_r3 = skew(traj.rl_foot_placement)
    skew_r4 = skew(traj.rr_foot_placement)
    
    g = np.array([
        [0],
        [0],
        [-9.81]
    ])



    psi_avg = np.average(traj.yaw_ref)
    R_z_T = np.arrary([
        [1, 0,             sin(psi_avg)],
        [0, cos(psi_avg), -sin(psi_avg)],
        [0, sin(psi_avg),  cos(psi_avg)]
    ])

    A_continuous = np.array([
        [np.zeros((3, 3)), np.zeros((3, 3)), R_z_T,            np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.eye(3),        np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 1))],
        [np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), (1/m)*g],
        [np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)), np.zeros((1, 3)),]
    ])

    x_vel = 0.5
    y_vel = 0
    z_pos = 0
    timeStep = 0.05
    timeHorizon = 5
    yawRate = 0

    traj = generateConstantTraj(state, x_vel, y_vel, z_pos, yawRate, timeStep, timeHorizon, 0.6, 0.7)
    N = timeHorizon/timeStep

    B_continuous = np.zeros((N, 13, 12))
    B = np.zeros((N, 13, 12))
    for i in range(N):



        r_x = []

        B_continuous[i] = np.array([
            [np.zeros(3, 3), np.zeros(3, 3), np.zeros(3, 3), np.zeros(3, 3)],
            [np.zeros(3, 3), np.zeros(3, 3), np.zeros(3, 3), np.zeros(3, 3)],
            [I_inv * ]
        ])
        

    
