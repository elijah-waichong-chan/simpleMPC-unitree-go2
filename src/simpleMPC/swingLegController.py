import numpy as np
from robot_classes import RigidBodyState, RigidBodyTraj, ConfigurationState

from pinocchioFunctions.readURDF import createFloatingBaseModel
from pinocchioFunctions.inverseKinematics import go2InverseKinematics

import pinocchio as pin 
model, data, vmodel, vdata, cmodel, cdata  = createFloatingBaseModel()

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

def computeFootJacobian(leg: str, r_i):

    config = ConfigurationState()   
    footID = model.getFrameId(f"{leg}_foot")
    q_guess = config.q

    q_new = go2InverseKinematics(f"{leg}", r_i, q_guess)
    J_world = pin.computeFrameJacobian(model, data, q_new, footID, pin.ReferenceFrame.WORLD)
    print(J_world)
    J_pos_world = J_world[:3,:]

    return J_pos_world
    
r_1 =  np.array([0.2, 0.22, -0.15])
computeFootJacobian('FL', r_1)