from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path
import pinocchio as pin 


def createFloatingBaseModel():
    repo = Path(__file__).resolve().parents[2]
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

    return model, data, vmodel, vdata, cmodel, cdata

if __name__ == "__main__":
    model, data, vmodel, vdata, cmodel, cdata  = createFloatingBaseModel()

    print("Loaded model:", model.name)

    print("# of Joints:", model.njoints)
    print("Names of Joints:", list(model.names))  

    print("# of Frames:", model.nframes)
    print("Names of Frames:", [f.name for f in model.frames[:model.nframes]])
