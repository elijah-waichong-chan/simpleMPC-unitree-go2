import pinocchio as pin 
from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path

import numpy as np
from numpy.linalg import norm, solve

import time
from pinocchio.visualize import MeshcatVisualizer

repo = Path(__file__).resolve().parents[3]
urdf_path = repo / "go2_description" / "urdf" / "go2_description.urdf"

# Create the Unitree Go2 Robot Object
robot = RobotWrapper.BuildFromURDF(
    str(urdf_path),
    package_dirs=[str(repo)],
    #root_joint=pin.JointModelFreeFlyer()
)

model = robot.model
vmodel = robot.visual_model
cmodel = robot.collision_model

data  = model.createData()
vdata  = pin.GeometryData(vmodel)
cdata  = pin.GeometryData(cmodel)


def go2InverseKinematics(leg: str, p_des_H: np.array):
    
    q0 = pin.neutral(model)
    q = q0

    eps = 0.1
    IT_MAX = 1000
    step = 0.2
    damp = 1e-7


    baseID = model.getFrameId("base")
    footID = model.getFrameId(f"{leg}_foot")

    success = False

    i = 0
    while True:
        pin.forwardKinematics(model,data, q)
        pin.updateFramePlacements(model, data)

        oMb = data.oMf[baseID]
        oMf = data.oMf[footID]

        bMf = oMb.actInv(oMf) #current foot placement wrt hip
        p_now_H = bMf.translation #current

        #print(f"{i}: pos = {p_now_H.T}")

        e_pos_H = p_des_H - p_now_H
        print(f"{i}: error = {e_pos_H.T}")

        if np.linalg.norm(e_pos_H) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break

        J_world = pin.computeFrameJacobian(model, data, q, footID, pin.ReferenceFrame.WORLD)
        J_pos_world = J_world[:3,:]
        J_pos_hip = oMb.rotation.T @ J_pos_world

        joint_ids  = [model.getJointId(f"{leg}_hip_joint"), 
                      model.getJointId(f"{leg}_thigh_joint"), 
                      model.getJointId(f"{leg}_calf_joint")]
        
        vcols = [model.joints[jid].idx_v for jid in joint_ids]
        qcols = [model.joints[jid].idx_q for jid in joint_ids]
        #print(qcols)

        J_leg = J_pos_hip[:, vcols] 

        H = J_leg.T @ J_leg + (damp**2) * np.eye(J_leg.shape[1])
        delta_q_leg = np.linalg.solve(H, J_leg.T @ e_pos_H)


        #print(f"{i}: delta = {delta_q_leg.T}")
        q[qcols] = q[qcols] + step * delta_q_leg

        # qmin = model.lowerPositionLimit
        # qmax = model.upperPositionLimit
        # q[qcols] = np.minimum(np.maximum(q[qcols], qmin[qcols]), qmax[qcols])

        i += 1

    if success:
        print("Convergence achieved!")
        print(f"\nresult: {q.flatten().tolist()}")
        print(f"\nfinal error: {e_pos_H.T}")
    else:
        print(
            "\n"
            "Warning: the iterative algorithm has not reached convergence "
            "to the desired precision"
        )


    return q




qnew = go2InverseKinematics("FL", np.array([0.2, 0.22, -0.15]))

# Visualization in Meshcat
viz = MeshcatVisualizer(model, cmodel, vmodel)
viz.initViewer()          # starts a local meshcat server
viz.loadViewerModel()     # resolves package:// URIs via package_dirs

viz.display(qnew)
viz.displayVisuals(True)       # toggle visual meshes
viz.displayCollisions(False)   # toggle collision shapes

# Keep the viewer alive
while True:
    time.sleep(0.1)