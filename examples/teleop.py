"""
Real-time teleop for Go2 using Convex MPC + gamepad/keyboard.

Usage:
    conda activate go2-convex-mpc
    cd go2-convex-mpc
    python examples/teleop.py                  # keyboard only
    python examples/teleop.py --gamepad        # gamepad (HORIPAD S)
    python examples/teleop.py --gamepad --scene terrain
"""
import os
os.environ["MPLBACKEND"] = "TkAgg"
import argparse
import time
import threading
import mujoco as mj
import mujoco.viewer as mjv
import numpy as np

from convex_mpc.go2_robot_data import PinGo2Model
from convex_mpc.mujoco_model import MuJoCo_GO2_Model
from convex_mpc.com_trajectory import ComTraj
from convex_mpc.centroidal_mpc import CentroidalMPC
from convex_mpc.leg_controller import LegController
from convex_mpc.gait import Gait

# ── Sim / control rates ──────────────────────────────────────────────────────
SIM_HZ   = 1000
SIM_DT   = 1.0 / SIM_HZ
CTRL_HZ  = 200
CTRL_DT  = 1.0 / CTRL_HZ
CTRL_DECIM = SIM_HZ // CTRL_HZ

GAIT_HZ  = 3.0
GAIT_DUTY = 0.6
GAIT_T   = 1.0 / GAIT_HZ
MPC_DT   = GAIT_T / 16
MPC_HZ   = 1.0 / MPC_DT
STEPS_PER_MPC = max(1, int(CTRL_HZ // MPC_HZ))

# ── Torque limits ─────────────────────────────────────────────────────────────
SAFETY = 0.9
TAU_LIM = SAFETY * np.array([
    23.7, 23.7, 45.43,   # FL
    23.7, 23.7, 45.43,   # FR
    23.7, 23.7, 45.43,   # RL
    23.7, 23.7, 45.43,   # RR
])
LEG_SLICE = {
    "FL": slice(0, 3), "FR": slice(3, 6),
    "RL": slice(6, 9), "RR": slice(9, 12),
}

# ── Velocity limits ───────────────────────────────────────────────────────────
VX_MAX     = 0.5
VY_MAX     = 0.3
YAW_MAX    = 1.5
Z_DEFAULT  = 0.27
Z_MIN      = 0.18
Z_MAX      = 0.35
DEADZONE   = 0.12
SMOOTH_ALPHA = 0.05

# ── Front leg PD override ────────────────────────────────────────────────────
FRONT_STAND = np.array([0.0, 0.9, -1.8])
FRONT_KP = 40.0
FRONT_KD = 4.0
FL_QPOS = slice(7, 10)
FR_QPOS = slice(10, 13)
FL_QVEL = slice(6, 9)
FR_QVEL = slice(9, 12)
FRONT_THIGH_MIN = -0.5
FRONT_THIGH_MAX = 1.5
FRONT_CALF_MIN  = -2.5
FRONT_CALF_MAX  = -0.5

# ── HORIPAD S gamepad mapping ─────────────────────────────────────────────────
AX_LX, AX_LY, AX_RX, AX_RY = 0, 1, 2, 3
BTN_Y, BTN_B, BTN_A, BTN_X = 0, 1, 2, 3
BTN_L, BTN_R = 4, 5
BTN_ZL, BTN_ZR = 6, 7
BTN_MINUS, BTN_PLUS = 8, 9
BTN_CAMERA = 10
BTN_HOME = 12


class TeleopState:
    def __init__(self):
        self.x_vel_raw = 0.0
        self.y_vel_raw = 0.0
        self.yaw_rate_raw = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.yaw_rate = 0.0
        self.z_pos = Z_DEFAULT
        self.reset_requested = False
        self.quit_requested = False
        self.front_override = False
        self.front_thigh_offset = 0.0
        self.front_calf_offset = 0.0
        self.front_hip_offset = 0.0
        self._lock = threading.Lock()

    def get_cmd(self):
        with self._lock:
            return self.x_vel, self.y_vel, self.z_pos, self.yaw_rate

    def smooth_update(self):
        with self._lock:
            a = SMOOTH_ALPHA
            self.x_vel    += a * (self.x_vel_raw    - self.x_vel)
            self.y_vel    += a * (self.y_vel_raw    - self.y_vel)
            self.yaw_rate += a * (self.yaw_rate_raw - self.yaw_rate)

    def get_front_override(self):
        with self._lock:
            if not self.front_override:
                return None
            target = FRONT_STAND.copy()
            target[0] += self.front_hip_offset
            target[1] += self.front_thigh_offset
            target[2] += self.front_calf_offset
            target[1] = np.clip(target[1], FRONT_THIGH_MIN, FRONT_THIGH_MAX)
            target[2] = np.clip(target[2], FRONT_CALF_MIN, FRONT_CALF_MAX)
            return target

    def consume_reset(self):
        with self._lock:
            if self.reset_requested:
                self.reset_requested = False
                return True
            return False


def apply_deadzone(v, dz=DEADZONE):
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def make_key_callback(state: TeleopState):
    def key_callback(keycode):
        with state._lock:
            k = keycode
            if k == 265 or k == 87:
                state.x_vel_raw = VX_MAX
            elif k == 264 or k == 83:
                state.x_vel_raw = -VX_MAX
            elif k == 263 or k == 65:
                state.yaw_rate_raw = YAW_MAX
            elif k == 262 or k == 68:
                state.yaw_rate_raw = -YAW_MAX
            elif k == 81:
                state.y_vel_raw = VY_MAX
            elif k == 69:
                state.y_vel_raw = -VY_MAX
            elif k == 32:
                state.x_vel_raw = 0.0
                state.y_vel_raw = 0.0
                state.yaw_rate_raw = 0.0
            elif k == 82:
                state.reset_requested = True
            elif k == 256:
                state.quit_requested = True
            elif k == 91:
                state.z_pos = max(Z_MIN, state.z_pos - 0.02)
            elif k == 93:
                state.z_pos = min(Z_MAX, state.z_pos + 0.02)
            elif k == 70:
                state.front_override = not state.front_override
                if not state.front_override:
                    state.front_thigh_offset = 0.0
                    state.front_calf_offset = 0.0
                    state.front_hip_offset = 0.0
            elif k == 73:
                state.front_thigh_offset -= 0.1
            elif k == 75:
                state.front_thigh_offset += 0.1
            elif k == 74:
                state.front_calf_offset += 0.1
            elif k == 76:
                state.front_calf_offset -= 0.1
    return key_callback


def gamepad_thread(state: TeleopState):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    pygame.display.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[teleop] No gamepad found.")
        return

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"[teleop] Gamepad: {js.get_name()}")

    while not state.quit_requested:
        pygame.event.pump()
        lx = apply_deadzone(js.get_axis(AX_LX))
        ly = apply_deadzone(js.get_axis(AX_LY))
        rx = apply_deadzone(js.get_axis(AX_RX))
        ry = apply_deadzone(js.get_axis(AX_RY))
        l_held = js.get_button(BTN_L)

        with state._lock:
            if l_held:
                state.front_override = True
                state.x_vel_raw  = -ly * VX_MAX * 0.5
                state.y_vel_raw  = -lx * VY_MAX * 0.5
                state.yaw_rate_raw = 0.0
                state.front_thigh_offset = -ry * 1.2
                state.front_hip_offset = rx * 0.3
                hat = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)
                state.front_calf_offset += hat[1] * 0.02
            else:
                if state.front_override:
                    state.front_override = False
                    state.front_thigh_offset = 0.0
                    state.front_calf_offset = 0.0
                    state.front_hip_offset = 0.0
                state.x_vel_raw    = -ly * VX_MAX
                state.y_vel_raw    = -lx * VY_MAX
                state.yaw_rate_raw = -rx * YAW_MAX

            if js.get_button(BTN_A):
                state.reset_requested = True
            if js.get_button(BTN_PLUS):
                state.quit_requested = True
            if js.get_button(BTN_ZL):
                state.z_pos = max(Z_MIN, state.z_pos - 0.002)
            if js.get_button(BTN_ZR):
                state.z_pos = min(Z_MAX, state.z_pos + 0.002)

        time.sleep(0.01)


def compute_front_pd_torque(mujoco_go2, target_q):
    q_fl = mujoco_go2.data.qpos[FL_QPOS]
    dq_fl = mujoco_go2.data.qvel[FL_QVEL]
    q_fr = mujoco_go2.data.qpos[FR_QPOS]
    dq_fr = mujoco_go2.data.qvel[FR_QVEL]
    target_fr = target_q.copy()
    target_fr[0] = -target_q[0]
    tau_fl = FRONT_KP * (target_q - q_fl) - FRONT_KD * dq_fl
    tau_fr = FRONT_KP * (target_fr - q_fr) - FRONT_KD * dq_fr
    return tau_fl, tau_fr


def main():
    parser = argparse.ArgumentParser(description="Go2 MPC Teleop")
    parser.add_argument("--gamepad", action="store_true")
    parser.add_argument("--scene", choices=["flat", "terrain"], default="flat")
    args = parser.parse_args()

    state = TeleopState()
    go2 = PinGo2Model()
    mujoco_go2 = MuJoCo_GO2_Model()

    if args.scene == "terrain":
        from pathlib import Path
        repo = Path(__file__).resolve().parents[1]
        terrain_xml = repo / "models" / "MJCF" / "go2" / "scene_terrain.xml"
        if terrain_xml.exists():
            mujoco_go2.model = mj.MjModel.from_xml_path(str(terrain_xml))
            mujoco_go2.data = mj.MjData(mujoco_go2.model)
            mujoco_go2.base_bid = mj.mj_name2id(
                mujoco_go2.model, mj.mjtObj.mjOBJ_BODY, "base_link"
            )

    mujoco_go2.model.opt.timestep = SIM_DT
    q_init = go2.current_config.get_q().copy()
    mujoco_go2.update_with_q_pin(q_init)

    gait = Gait(GAIT_HZ, GAIT_DUTY)
    leg_controller = LegController()
    traj = ComTraj(go2)
    traj.generate_traj(go2, gait, 0.0, 0.0, 0.0, Z_DEFAULT, 0.0, time_step=MPC_DT)
    mpc = CentroidalMPC(go2, traj)

    U_opt = np.zeros((12, traj.N), dtype=float)
    tau_hold = np.zeros(12, dtype=float)

    key_cb = make_key_callback(state)
    viewer = mjv.launch_passive(
        mujoco_go2.model, mujoco_go2.data,
        key_callback=lambda k: key_cb(k),
    )
    viewer.cam.type = mj.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = mujoco_go2.base_bid
    viewer.cam.distance = 2.0
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 90
    viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = True

    if args.gamepad:
        threading.Thread(target=gamepad_thread, args=(state,), daemon=True).start()

    print("\n=== Go2 MPC Teleop ===")
    if args.gamepad:
        print("  Left stick       : forward/back + strafe")
        print("  Right stick X    : turn left/right")
        print("  ZL / ZR          : lower / raise body")
        print("  A button         : reset sim")
        print("  + button         : quit")
        print("  Hold L bumper    : front leg mode (R stick = thigh/hip)")
    else:
        print("  W/S              : forward / back")
        print("  A/D              : turn left / right")
        print("  Q/E              : strafe left / right")
        print("  [ / ]            : lower / raise body")
        print("  Space            : stop all motion")
        print("  R                : reset sim")
        print("  F                : toggle front leg mode")
        print("  I/K J/L          : front thigh / calf")
    print()

    ctrl_i = 0
    hud_counter = 0
    sim_step = 0

    while viewer.is_running() and not state.quit_requested:
        step_wall = time.perf_counter()
        time_now_s = float(mujoco_go2.data.time)

        if state.consume_reset():
            go2_fresh = PinGo2Model()
            go2.__dict__.update(go2_fresh.__dict__)
            mujoco_go2.update_with_q_pin(q_init)
            mujoco_go2.data.qvel[:] = 0
            mujoco_go2.data.time = 0.0
            mj.mj_forward(mujoco_go2.model, mujoco_go2.data)
            tau_hold[:] = 0
            ctrl_i = 0
            sim_step = 0
            gait = Gait(GAIT_HZ, GAIT_DUTY)
            leg_controller = LegController()
            traj = ComTraj(go2)
            traj.generate_traj(go2, gait, 0.0, 0.0, 0.0, Z_DEFAULT, 0.0, time_step=MPC_DT)
            mpc = CentroidalMPC(go2, traj)
            U_opt = np.zeros((12, traj.N), dtype=float)
            with state._lock:
                state.front_thigh_offset = 0.0
                state.front_calf_offset = 0.0
                state.front_hip_offset = 0.0
                state.front_override = False
                state.x_vel = state.y_vel = state.yaw_rate = 0.0
                state.x_vel_raw = state.y_vel_raw = state.yaw_rate_raw = 0.0
            print("\r[teleop] Reset!                    ")
            continue

        if (sim_step % CTRL_DECIM) == 0:
            state.smooth_update()
            x_vel, y_vel, z_pos, yaw_rate = state.get_cmd()
            gait.standing = (abs(x_vel) < 0.03 and abs(y_vel) < 0.03 and abs(yaw_rate) < 0.03)
            mujoco_go2.update_pin_with_mujoco(go2)

            if (ctrl_i % STEPS_PER_MPC) == 0:
                traj.generate_traj(go2, gait, time_now_s, x_vel, y_vel, z_pos, yaw_rate, time_step=MPC_DT)
                sol = mpc.solve_QP(go2, traj, False)
                N = traj.N
                w_opt = sol["x"].full().flatten()
                U_opt = w_opt[12 * N:].reshape((12, N), order="F")

            mpc_force = U_opt[:, 0]
            tau_raw = np.zeros(12)
            for leg in ("FL", "FR", "RL", "RR"):
                s = LEG_SLICE[leg]
                result = leg_controller.compute_leg_torque(leg, go2, gait, mpc_force[s], time_now_s)
                tau_raw[s] = result.tau

            tau_hold = np.clip(tau_raw, -TAU_LIM, TAU_LIM)

            front_target = state.get_front_override()
            if front_target is not None:
                tau_fl, tau_fr = compute_front_pd_torque(mujoco_go2, front_target)
                tau_hold[LEG_SLICE["FL"]] = np.clip(tau_fl, -TAU_LIM[:3], TAU_LIM[:3])
                tau_hold[LEG_SLICE["FR"]] = np.clip(tau_fr, -TAU_LIM[3:6], TAU_LIM[3:6])

            ctrl_i += 1
            hud_counter += 1
            if hud_counter % 20 == 0:
                mode = "FRONT" if state.front_override else "WALK "
                print(f"\r  [{mode}] vx={x_vel:+.2f} vy={y_vel:+.2f} yaw={yaw_rate:+.2f} z={z_pos:.3f} t={time_now_s:.1f}s  ", end="", flush=True)

        mj.mj_step1(mujoco_go2.model, mujoco_go2.data)
        mujoco_go2.set_joint_torque(tau_hold)
        mj.mj_step2(mujoco_go2.model, mujoco_go2.data)
        sim_step += 1

        if sim_step % (SIM_HZ // 60) == 0:
            viewer.sync()

        elapsed = time.perf_counter() - step_wall
        sleep_t = SIM_DT - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    viewer.close()
    print("\n[teleop] Done.")


if __name__ == "__main__":
    main()
