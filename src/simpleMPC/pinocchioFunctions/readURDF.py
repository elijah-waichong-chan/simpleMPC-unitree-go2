from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path
import pinocchio as pin 


def createFloatingBaseModel():
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

    return model, data, vmodel, vdata, cmodel, cdata

def getFootPlacement(model, data, q):

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    FL_id = model.getFrameId("FL_foot")
    FR_id = model.getFrameId("FR_foot")
    RL_id = model.getFrameId("RL_foot")
    RR_id = model.getFrameId("RR_foot")

    FL_placement = data.oMf[FL_id].translation
    FR_placement = data.oMf[FR_id].translation
    RL_placement = data.oMf[RL_id].translation
    RR_placement = data.oMf[RR_id].translation

    return FL_placement, FR_placement, RL_placement, RR_placement

def getHipOffset(model, data, q):

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)

    FL_id = model.getFrameId("FL_thigh_joint")
    FR_id = model.getFrameId("FR_thigh_joint")
    RL_id = model.getFrameId("RL_thigh_joint")
    RR_id = model.getFrameId("RR_thigh_joint")

    FL_hip_placement = data.oMf[FL_id].translation
    FR_hip_placement = data.oMf[FR_id].translation
    RL_hip_placement = data.oMf[RL_id].translation
    RR_hip_placement = data.oMf[RR_id].translation

    return FL_hip_placement, FR_hip_placement, RL_hip_placement, RR_hip_placement



if __name__ == "__main__":
    model, data, vmodel, vdata, cmodel, cdata  = createFloatingBaseModel()
    
    q = pin.neutral(model)
    [FL_placement, FR_placement, RL_placement, RR_placement] = getFootPlacement(model, data, q)
    [fl_hip_offset, fr_hip_offset, rl_hip_offset, rr_hip_offset] = getHipOffset(model, data, q)
    print(fl_hip_offset)
    print(FL_placement)