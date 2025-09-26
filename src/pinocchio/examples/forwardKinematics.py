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

# Forward Kinematic Code Below

q = pin.neutral(model)  # q = 0

pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)


# Now you can query the placement of a joint or frame:
frame_id = model.getFrameId("FL_hip_joint")
joint_id = model.getFrameId("FL_hip_joint")
print("Placement:\n", data.oMf[frame_id])