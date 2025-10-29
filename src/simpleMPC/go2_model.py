from pinocchio.robot_wrapper import RobotWrapper
from pathlib import Path
import pinocchio as pin 
from robot_classes import ConfigurationState
import numpy as np

class Go2Model:

    def __init__(self):
        # Locate URDF relative to this file
        repo = Path(__file__).resolve().parents[2]
        urdf_path = repo / "go2_description" / "urdf" / "go2_description.urdf"

        # Build robot (free-flyer at the root)
        robot = RobotWrapper.BuildFromURDF(
            str(urdf_path),
            package_dirs=[str(repo)],
            root_joint=pin.JointModelFreeFlyer()
        )

        # Core models
        self.model = robot.model
        self.vmodel = robot.visual_model
        self.cmodel = robot.collision_model

        # Initial data containers
        self.data = self.model.createData()

        # Initial configuration
        config = ConfigurationState()
        q_init = config.compute_q()

        # Forward kinematics / frame placements at q_init
        pin.forwardKinematics(self.model, self.data, q_init)
        pin.updateFramePlacements(self.model, self.data)

        # Define a fixed "my_world" frame coincident with the current base pose
        self.base_id = self.model.getFrameId("base")
        base_pose_world = self.data.oMf[self.base_id].copy()
        my_world = pin.Frame(
            "my_world",
            0,                          # parent joint: universe
            0,                          # parent frame (unused for OP_FRAME)
            base_pose_world,            # absolute placement right now
            pin.FrameType.OP_FRAME
        )

        self.my_world_id = self.model.addFrame(my_world)

        self.FL_foot_id = self.model.getFrameId("FL_foot")
        self.FR_foot_id = self.model.getFrameId("FR_foot")
        self.RL_foot_id = self.model.getFrameId("RL_foot")
        self.RR_foot_id = self.model.getFrameId("RR_foot")

        self.FL_hip_id = self.model.getFrameId("FL_thigh_joint")
        self.FR_hip_id = self.model.getFrameId("FR_thigh_joint")
        self.RL_hip_id = self.model.getFrameId("RL_thigh_joint")
        self.RR_hip_id = self.model.getFrameId("RR_thigh_joint")

        # Update Data containers
        self.data = self.model.createData()
        self.vdata = pin.GeometryData(self.vmodel)
        self.cdata = pin.GeometryData(self.cmodel)

        oMw = self.data.oMf[self.my_world_id]
        oMh1 = self.data.oMf[self.FL_hip_id]
        oMh2 = self.data.oMf[self.FR_hip_id]
        oMh3 = self.data.oMf[self.RL_hip_id]
        oMh4 = self.data.oMf[self.RR_hip_id]

        wMh1 = oMw.actInv(oMh1)
        wMh2 = oMw.actInv(oMh2)
        wMh3 = oMw.actInv(oMh3)
        wMh4 = oMw.actInv(oMh4)

        self.FL_hip_offset = wMh1.translation.copy()
        self.FR_hip_offset = wMh2.translation.copy()
        self.RL_hip_offset = wMh3.translation.copy()
        self.RR_hip_offset = wMh4.translation.copy()



    def test(self):
        oMb = self.data.oMf[self.base_id]
        p_ob = oMb.translation.copy() 
        return p_ob                        

    def get_foot_placement_in_world(self, config: ConfigurationState):

        q = config.compute_q()

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        oMw = self.data.oMf[self.my_world_id]
        oMf1 = self.data.oMf[self.FL_foot_id]
        oMf2 = self.data.oMf[self.FR_foot_id]
        oMf3 = self.data.oMf[self.RL_foot_id]
        oMf4 = self.data.oMf[self.RR_foot_id]

        wMf1 = oMw.actInv(oMf1)
        wMf2 = oMw.actInv(oMf2)
        wMf3 = oMw.actInv(oMf3)
        wMf4 = oMw.actInv(oMf4)


        FL_placement = wMf1.translation.copy()
        FR_placement = wMf2.translation.copy()
        RL_placement = wMf3.translation.copy()
        RR_placement = wMf4.translation.copy()

        return FL_placement, FR_placement, RL_placement, RR_placement
    
    def get_foot_placement_in_body(self, config: ConfigurationState):

        q = config.compute_q()

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        oMb = self.data.oMf[self.base_id]
        oMf1 = self.data.oMf[self.FL_foot_id]
        oMf2 = self.data.oMf[self.FR_foot_id]
        oMf3 = self.data.oMf[self.RL_foot_id]
        oMf4 = self.data.oMf[self.RR_foot_id]

        wMf1 = oMb.actInv(oMf1)
        wMf2 = oMb.actInv(oMf2)
        wMf3 = oMb.actInv(oMf3)
        wMf4 = oMb.actInv(oMf4)

        FL_placement = wMf1.translation.copy()
        FR_placement = wMf2.translation.copy()
        RL_placement = wMf3.translation.copy()
        RR_placement = wMf4.translation.copy()

        return FL_placement, FR_placement, RL_placement, RR_placement
    
    def inverse_kinematics(self, leg: str, p_des_H: np.array, config: ConfigurationState):
    
        q = config.compute_q()

        eps = 0.1
        IT_MAX = 1000
        step = 0.2
        damp = 1e-7

        footID = self.model.getFrameId(f"{leg}_foot")

        success = False

        i = 0
        while True:
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            oMb = self.data.oMf[self.base_id]
            oMf = self.data.oMf[footID]

            bMf = oMb.actInv(oMf)
            p_now_H = bMf.translation

            #print(f"{i}: pos = {p_now_H.T}")

            e_pos_H = p_des_H - p_now_H
            print(f"{i}: error = {e_pos_H.T}")

            if np.linalg.norm(e_pos_H) < eps:
                success = True
                break
            if i >= IT_MAX:
                success = False
                break

            J_world = pin.computeFrameJacobian(self.model, self.data, q, footID, pin.ReferenceFrame.WORLD)
            J_pos_world = J_world[:3,:]
            J_pos_hip = oMb.rotation.T @ J_pos_world

            joint_ids  = [self.model.getJointId(f"{leg}_hip_joint"), 
                          self.model.getJointId(f"{leg}_thigh_joint"), 
                          self.model.getJointId(f"{leg}_calf_joint")]
            
            vcols = [self.model.joints[jid].idx_v for jid in joint_ids]
            qcols = [self.model.joints[jid].idx_q for jid in joint_ids]
            #print(qcols)

            J_leg = J_pos_hip[:, vcols] 

            H = J_leg.T @ J_leg + (damp**2) * np.eye(J_leg.shape[1])
            delta_q_leg = np.linalg.solve(H, J_leg.T @ e_pos_H)

            q[qcols] = q[qcols] + step * delta_q_leg

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

        config.update_config(q)

        return config



