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