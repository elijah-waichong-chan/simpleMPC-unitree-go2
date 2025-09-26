import pinocchio as pin 
from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path

repo = Path(__file__).resolve().parents[3]
urdf_path = repo / "go2_description" / "urdf" / "go2_description.urdf"

# Create the Unitree Go2 Robot Object
robot = RobotWrapper.BuildFromURDF(
    str(urdf_path),
    package_dirs=[str(repo)]   # repo contains the folder "GO2_URDF/"
)

model = robot.model
data  = model.createData()

# Desired generalized coordinate
q = pin.neutral(model)  # q = 0

# Compute the joint pose
pin.forwardKinematics(model, data, q)
# Update the frame pose
pin.updateFramePlacements(model, data)


# Now the joint and frame pose have been updated
frame_id = model.getFrameId("FL_hip_joint")
joint_id = model.getJointId("FL_hip_joint")
print("Frame Placement:\n", data.oMf[frame_id])
print("Joint Placement:\n", data.oMi[joint_id])
