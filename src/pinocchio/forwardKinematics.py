import pinocchio as pin 
from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path

import numpy as np
from pinocchio.vis_mujoco import visualize

repo = Path(__file__).resolve().parents[3]
urdf_path = repo / "go2_description" / "urdf" / "go2_description.urdf"

# Create the Unitree Go2 Robot Object
robot = RobotWrapper.BuildFromURDF(
    str(urdf_path),
    package_dirs=[str(repo)],
    root_joint=pin.JointModelFreeFlyer()
)

model = robot.model
vmodel = robot.visual_model
cmodel = robot.collision_model

data  = model.createData()
vdata  = pin.GeometryData(vmodel)
cdata  = pin.GeometryData(cmodel)

# Desired generalized coordinate
x, y, z, =  0, 0, 0                     # base position (m)
qx, qy, qz, qw = 0, 0, 0, 1             # base orientation  (quaternion)
theta1, theta2, theta3 = 1, 0, 0        # leg 1 joint angle (rad)
theta4, theta5, theta6 = 0, 0, 0        # leg 2 joint angle (rad)
theta7, theta8, theta9 = 0, 0, 0        # leg 3 joint angle (rad)
theta10, theta11, theta12 = 0, 0, 0     # leg 4 joint angle (rad)

q = np.array([x, y, z,                      
              qx, qy, qz, qw,                 
              theta1, theta2, theta3,
              theta4, theta5, theta6,
              theta7, theta8, theta9,
              theta10, theta11, theta12]) 

#q = pin.neutral(model).copy()

print("q:\n", q)

 

# Compute the joint pose
pin.forwardKinematics(model, data, q)
# Update the frame pose
pin.updateFramePlacements(model, data)
pin.updateGeometryPlacements(model, data, vmodel, vdata)
pin.updateGeometryPlacements(model, data, cmodel, cdata)


# Now the joint and frame pose have been updatedutils
frame_id = model.getFrameId("FL_foot")
frame_id2 = model.getFrameId("FL_hip")
joint_id = model.getJointId("FL_hip_joint")
# print("Foot Placement:\n", data.oMf[frame_id])
# print("Hip Placement:\n", data.oMf[frame_id2])
# print("Joint Placement:\n", data.oMi[joint_id])


visualize(model, q)