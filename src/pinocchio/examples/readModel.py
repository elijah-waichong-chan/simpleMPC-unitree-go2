from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path

repo = Path(__file__).resolve().parents[3]
urdf_path = repo / "go2_description" / "urdf" / "go2_description.urdf"  # confirm this name

robot = RobotWrapper.BuildFromURDF(
    str(urdf_path),
    package_dirs=[str(repo)]   # repo contains the folder "GO2_URDF/"
)

model = robot.model

print("Loaded model:", model.name)

print("# of Joints:", model.njoints)
print("Names of Joints:", list(model.names))  

print("# of Frames:", model.nframes)
print("Names of Frames:", [f.name for f in model.frames[:model.nframes]])
