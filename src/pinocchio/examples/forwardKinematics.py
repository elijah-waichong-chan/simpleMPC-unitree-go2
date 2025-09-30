import pinocchio as pin 
from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path

import numpy as np

import time
from pinocchio.visualize import MeshcatVisualizer

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
x, y, z, =  0, 0, 1
qx, qy, qz, qw = 0, 0, 0, 1
theta1, theta2, theta3 = 0, 0, 0 
theta4, theta5, theta6 = 0, 0, 0 
theta7, theta8, theta9 = 0, 0, 0 
theta10, theta11, theta12 = 0, 0, 0 


q = np.array([x, y, z,                      # base position
             qx, qy, qz, qw,                 # base orientation  (quaternion)
             theta1, theta2, theta3,
             theta4, theta5, theta6,
             theta7, theta8, theta9,
             theta10, theta11, theta12])    # leg joints

#q = pin.neutral(model).copy()

print("q:\n", q)

 

# Compute the joint pose
pin.forwardKinematics(model, data, q)
# Update the frame pose
pin.updateFramePlacements(model, data)
pin.updateGeometryPlacements(model, data, vmodel, vdata)
pin.updateGeometryPlacements(model, data, cmodel, cdata)


# Now the joint and frame pose have been updated
frame_id = model.getFrameId("FL_hip_joint")
joint_id = model.getJointId("FL_hip_joint")
print("Frame Placement:\n", data.oMf[frame_id])
print("Joint Placement:\n", data.oMi[joint_id])


# Visualization in Meshcat
viz = MeshcatVisualizer(model, cmodel, vmodel)
viz.initViewer()          # starts a local meshcat server
viz.loadViewerModel()     # resolves package:// URIs via package_dirs

viz.display(q)
viz.displayVisuals(True)       # toggle visual meshes
viz.displayCollisions(False)   # toggle collision shapes

# Keep the viewer alive
while True:
    time.sleep(0.1)