import time
from pathlib import Path

import mujoco as mj
import mujoco.viewer
import mujoco.viewer as mjv
import numpy as np
import pinocchio as pin

from .go2_robot_data import PinGo2Model


# --------------------------------------------------------------------------------
# MuJoCo Model Setting
# --------------------------------------------------------------------------------

REPO = Path(__file__).resolve().parents[2]
XML_PATH = REPO / "models" / "MJCF" / "go2" / "scene.xml"


class MuJoCo_GO2_Model:
    def __init__(self):
        # Load the MuJoCo model
        self.model = mj.MjModel.from_xml_path(str(XML_PATH))
        self.data = mj.MjData(self.model)
        self.viewer = None
        self.base_bid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, "base_link")

    def update_with_q_pin(self, q_pin):
        px, py, pz, qx, qy, qz, qw, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12 = q_pin[:]
        self.data.qpos[:] = [
            px,
            py,
            pz,
            qw,
            qx,
            qy,
            qz,
            t1,
            t2,
            t3,
            t4,
            t5,
            t6,
            t7,
            t8,
            t9,
            t10,
            t11,
            t12,
        ]
        mj.mj_forward(self.model, self.data)

    def set_leg_joint_torque(self, leg: str, torque):
        # Get joint ID
        aid_hip = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_hip")
        aid_thigh = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_thigh")
        aid_calf = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, f"{leg}_calf")

        # Set joint torque
        self.data.ctrl[aid_hip] = torque[0]
        self.data.ctrl[aid_thigh] = torque[1]
        self.data.ctrl[aid_calf] = torque[2]

    def set_joint_torque(self, torque):
        # Set joint torque
        self.set_leg_joint_torque("FL", torque[0:3])
        self.set_leg_joint_torque("FR", torque[3:6])
        self.set_leg_joint_torque("RL", torque[6:9])
        self.set_leg_joint_torque("RR", torque[9:12])

    def update_pin_with_mujoco(self, go2: PinGo2Model):

        # Mujoco q has different ordering from the pinocchio q
        # In addition, mujoco dq stores velocity in world frame, while pinocchio stores in body frame
        # Both mujoco and pinocchio stores body frame angular velocity

        mujoco_q = np.asarray(self.data.qpos, dtype=float).reshape(-1)  # (19,)
        mujoco_dq = np.asarray(self.data.qvel, dtype=float).reshape(-1)  # (18,)
        qw, qx, qy, qz = mujoco_q[3:7]
        # pin.Quaternion uses qw qx qy qz
        R = pin.Quaternion(qw, qx, qy, qz).toRotationMatrix()  # body -> world
        v_world = mujoco_dq[0:3]
        w_body = mujoco_dq[3:6]
        v_body = R.T @ v_world

        # Convert to Pin
        # configuration uses qx qy qz qw
        q_pin = np.concatenate([mujoco_q[0:3], [qx, qy, qz, qw], mujoco_q[7:]])
        dq_pin = np.concatenate([v_body, w_body, mujoco_dq[6:]])

        go2.update_model(q_pin, dq_pin)

    def replay_simulation(self, time_log_s, q_log, tau_log_Nm, RENDER_DT, REALTIME_FACTOR):
        model = self.model
        data_replay = mj.MjData(model)

        with mjv.launch_passive(model, data_replay) as viewer:

            # 1) Pick the body to track (change "base" to your torso/body name)
            base_id = model.body("base_link").id  # or e.g. model.body("torso").id

            # 2) Configure camera as a tracking camera
            viewer.cam.type = mj.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = base_id
            viewer.cam.fixedcamid = -1  # not using a fixed camera slot

            # Optional: nice initial view
            viewer.cam.distance = 2.0  # how far from the body
            viewer.cam.elevation = -20  # vertical angle (deg)
            viewer.cam.azimuth = 90  # horizontal angle (deg)

            viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = True

            while viewer.is_running():  # loop until the window is closed

                start_wall = time.perf_counter()
                t0 = time_log_s[0]
                next_render_t = t0

                k = 0
                T = len(time_log_s)

                # One full replay
                while k < T and viewer.is_running():
                    t = time_log_s[k]

                    # time to render a frame?
                    if t >= next_render_t:
                        data_replay.qpos[:] = q_log[k]
                        data_replay.ctrl[:] = tau_log_Nm[k]
                        mj.mj_forward(model, data_replay)
                        viewer.sync()

                        # real-time pacing
                        target_wall = start_wall + (t - t0) / REALTIME_FACTOR
                        now = time.perf_counter()
                        sleep_time = (
                            target_wall - now
                        )  # 2) Compute the dynamics and desir ed trajectory

                        if sleep_time > 0:
                            time.sleep(sleep_time)

                        next_render_t += RENDER_DT

                    k += 1

                time.sleep(1)
