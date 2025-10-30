import mujoco as mj
import mujoco.viewer
import numpy as np
from pathlib import Path
import time

class MuJoCo_GO2_Model:
    def __init__(self):
        # Set this to your MJCF file directory
        repo = Path(__file__).resolve().parents[2]
        scene_path = repo / "unitree_mujoco" / "unitree_robots" / "go2" / "scene.xml"

        # Load the MuJoCo model
        self.model = mj.MjModel.from_xml_path(str(scene_path))
        self.data = mj.MjData(self.model)
        self.viewer = None

    def update_with_q_pin(self, q_pin):
        px, py, pz, qx, qy, qz, qw, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12 = q_pin[:]
        self.data.qpos[:] = [px, py, pz, qw, qx, qy, qz, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12]
        mj.mj_forward(self.model, self.data)   # recompute derived quantities

    def start_viewer(self):
        self.viewer = mj.viewer.launch_passive(self.model, self.data).__enter__()
        self.viewer.sync()

    def hold_viewer(self):
        while self.viewer.is_running():
                #mujoco.mj_step(self.model, self.data) #Comment this line for passive viewer
                self.viewer.sync()
        


    def set_leg_joint_torque(self, leg: str, torque):

        aid_hip = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_hip")
        aid_thigh = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_thigh")
        aid_calf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_calf")

        self.data.ctrl[aid_hip] = torque[0]
        self.data.ctrl[aid_thigh] = torque[1]
        self.data.ctrl[aid_calf] = torque[2]

    def get_leg_joint_pos(self, leg: str):

        q_leg = np.zeros(3, dtype=float)

        jid_hip = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{leg}_hip_joint")
        jid_thigh = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{leg}_thigh_joint")
        jid_calf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{leg}_calf_joint")

        q_idx_hip = self.model.jnt_qposadr[jid_hip]
        q_idx_thigh = self.model.jnt_qposadr[jid_thigh]
        q_idx_calf = self.model.jnt_qposadr[jid_calf]

        q_leg[0] = float(self.data.qpos[q_idx_hip])
        q_leg[1] = float(self.data.qpos[q_idx_thigh])
        q_leg[2] = float(self.data.qpos[q_idx_calf])

        return q_leg
    
    def get_leg_joint_vel(self, leg: str):

        v_leg = np.zeros(3, dtype=float)

        jid_hip = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{leg}_hip_joint")
        jid_thigh = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{leg}_thigh_joint")
        jid_calf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_JOINT, f"{leg}_calf_joint")

        v_idx_hip = self.model.jnt_dofadr[jid_hip]
        v_idx_thigh = self.model.jnt_dofadr[jid_thigh]
        v_idx_calf = self.model.jnt_dofadr[jid_calf]

        v_leg[0] = float(self.data.qpos[v_idx_hip])
        v_leg[1] = float(self.data.qpos[v_idx_thigh])
        v_leg[2] = float(self.data.qpos[v_idx_calf])

        return v_leg
    
    def run_sim(self, torque, control_hz):

        self.model.opt.timestep = 1.0 / control_hz
        dt = self.model.opt.timestep
        render_hz  = 60
        steps_per_render = max(1, int(control_hz // render_hz))
        next_t = time.perf_counter()

        self.start_viewer()
        i = 0
        print(f"Running simulation at {control_hz} Hz (dt={dt:.6f}s)...")

        while self.viewer.is_running():
            mj.mj_step1(self.model, self.data)
            self.set_leg_joint_torque("FL", torque[0:2])
            self.set_leg_joint_torque("FR", torque[3:5])
            self.set_leg_joint_torque("RL", torque[6:8])
            self.set_leg_joint_torque("RR", torque[9:11])
            mj.mj_step2(self.model, self.data)

            if (i % steps_per_render) == 0:
                self.viewer.sync()

            next_t += dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # fell behind, reset clock
                next_t = time.perf_counter()

            i += 1








