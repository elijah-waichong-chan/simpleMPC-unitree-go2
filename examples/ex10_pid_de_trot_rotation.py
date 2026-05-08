"""
Demo 10: PID-DE-MPC — Trot rotation.

Replaces CentroidalMPC with CentroidalPIDDEMPC (event-triggered + ESO).
"""

import os


os.environ["MPLBACKEND"] = "Agg"
import time
from dataclasses import dataclass, field

import mujoco as mj
import numpy as np

from convex_mpc.com_trajectory import ComTraj
from convex_mpc.gait import Gait
from convex_mpc.go2_robot_data import PinGo2Model
from convex_mpc.leg_controller import LegController
from convex_mpc.mujoco_model import MuJoCo_GO2_Model
from convex_mpc.pid_de_mpc import CentroidalPIDDEMPC
from convex_mpc.plot_helper import (
    hold_until_all_fig_closed,
    plot_mpc_result,
    plot_solve_time,
    plot_swing_foot_traj,
)


LABEL = "ex10_pid_de_trot_rotation"

# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
INITIAL_X_POS = -5
INITIAL_Y_POS = 0
RUN_SIM_LENGTH_S = 5.0

RENDER_HZ = 120.0
RENDER_DT = 1.0 / RENDER_HZ
REALTIME_FACTOR = 1


@dataclass
class BodyCmdPhase:
    t_start: float
    t_end: float
    x_vel: float
    y_vel: float
    z_pos: float
    yaw_rate: float


CMD_SCHEDULE = [
    BodyCmdPhase(0.0, 5.0, 0.0, 0.0, 0.27, 4.0),
]

GAIT_HZ = 3
GAIT_DUTY = 0.6
GAIT_T = 1.0 / GAIT_HZ

x_vel_des_body = 0.0
y_vel_des_body = 0.0
z_pos_des_body = 0.27
yaw_rate_des_body = 0.0

SIM_HZ = 1000
SIM_DT = 1.0 / SIM_HZ

CTRL_HZ = 200
CTRL_DT = 1.0 / CTRL_HZ

if SIM_HZ % CTRL_HZ != 0:
    raise ValueError(f"SIM_HZ ({SIM_HZ}) must be divisible by CTRL_HZ ({CTRL_HZ})")
CTRL_DECIM = SIM_HZ // CTRL_HZ

SIM_STEPS = int(RUN_SIM_LENGTH_S * SIM_HZ)
CTRL_STEPS = int(RUN_SIM_LENGTH_S * CTRL_HZ)

MPC_DT = GAIT_T / 16
MPC_HZ = 1.0 / MPC_DT
STEPS_PER_MPC = max(1, int(CTRL_HZ // MPC_HZ))

HIP_LIM = 23.7
ABD_LIM = 23.7
KNEE_LIM = 45.43
SAFETY = 0.9

TAU_LIM = SAFETY * np.array(
    [
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,
    ]
)

LEG_SLICE = {
    "FL": slice(0, 3),
    "FR": slice(3, 6),
    "RL": slice(6, 9),
    "RR": slice(9, 12),
}


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------
def get_body_cmd(t: float):
    for phase in CMD_SCHEDULE:
        if phase.t_start <= t < phase.t_end:
            return phase.x_vel, phase.y_vel, phase.z_pos, phase.yaw_rate
    return 0.0, 0.0, 0.27, 0.0


# ---------------------------------------------------------------------------
# Storage
# ---------------------------------------------------------------------------
x_vec = np.zeros((12, CTRL_STEPS))
mpc_force_world = np.zeros((12, CTRL_STEPS))
tau_raw = np.zeros((12, CTRL_STEPS))
tau_cmd = np.zeros((12, CTRL_STEPS))

time_log_ctrl_s = np.zeros(CTRL_STEPS)
q_log_ctrl = np.zeros((CTRL_STEPS, 19))
tau_log_ctrl_Nm = np.zeros((CTRL_STEPS, 12))


@dataclass
class FootTraj:
    pos_des: np.ndarray = field(default_factory=lambda: np.zeros((12, CTRL_STEPS)))
    pos_now: np.ndarray = field(default_factory=lambda: np.zeros((12, CTRL_STEPS)))
    vel_des: np.ndarray = field(default_factory=lambda: np.zeros((12, CTRL_STEPS)))
    vel_now: np.ndarray = field(default_factory=lambda: np.zeros((12, CTRL_STEPS)))


foot_traj = FootTraj()

mpc_update_time_ms = []
mpc_solve_time_ms = []
X_opt = None
U_opt = None

# ---------------------------------------------------------------------------
# Initialisation
# ---------------------------------------------------------------------------
go2 = PinGo2Model()
mujoco_go2 = MuJoCo_GO2_Model()
leg_controller = LegController()
traj = ComTraj(go2)
gait = Gait(GAIT_HZ, GAIT_DUTY)

q_init = go2.current_config.get_q()
q_init[0], q_init[1] = INITIAL_X_POS, INITIAL_Y_POS
mujoco_go2.update_with_q_pin(q_init)
mujoco_go2.model.opt.timestep = SIM_DT

traj.generate_traj(
    go2,
    gait,
    0.0,
    x_vel_des_body,
    y_vel_des_body,
    z_pos_des_body,
    yaw_rate_des_body,
    time_step=MPC_DT,
)
mpc = CentroidalPIDDEMPC(go2, traj, mpc_dt=MPC_DT)

U_opt = np.zeros((12, traj.N), dtype=float)

time_log_render = []
q_log_render = []
tau_log_render = []
next_render_t = 0.0

# ---------------------------------------------------------------------------
# Simulation Loop
# ---------------------------------------------------------------------------
print(f"Running PID-DE-MPC simulation for {RUN_SIM_LENGTH_S}s")
sim_start_time = time.perf_counter()

ctrl_i = 0
tau_hold = np.zeros(12, dtype=float)

for k in range(SIM_STEPS):
    time_now_s = float(mujoco_go2.data.time)

    if (k % CTRL_DECIM) == 0 and ctrl_i < CTRL_STEPS:
        x_vel_des_body, y_vel_des_body, z_pos_des_body, yaw_rate_des_body = get_body_cmd(time_now_s)

        mujoco_go2.update_pin_with_mujoco(go2)
        x_vec[:, ctrl_i] = go2.compute_com_x_vec().reshape(-1)

        time_log_ctrl_s[ctrl_i] = time_now_s
        q_log_ctrl[ctrl_i, :] = mujoco_go2.data.qpos

        if (ctrl_i % STEPS_PER_MPC) == 0:
            print(f"\rSimulation Time: {time_now_s:.3f} s", end="", flush=True)

            traj.generate_traj(
                go2,
                gait,
                time_now_s,
                x_vel_des_body,
                y_vel_des_body,
                z_pos_des_body,
                yaw_rate_des_body,
                time_step=MPC_DT,
            )

            sol = mpc.solve_QP(go2, traj, False)
            mpc_solve_time_ms.append(mpc.solve_time)
            mpc_update_time_ms.append(mpc.update_time)

            N = traj.N
            w_opt = sol["x"].full().flatten()
            X_opt = w_opt[: 12 * N].reshape((12, N), order="F")
            U_opt = w_opt[12 * N :].reshape((12, N), order="F")

        mpc_force_world[:, ctrl_i] = U_opt[:, 0]

        for leg_name in ["FL", "FR", "RL", "RR"]:
            result = leg_controller.compute_leg_torque(
                leg_name, go2, gait, mpc_force_world[LEG_SLICE[leg_name], ctrl_i], time_now_s
            )
            tau_raw[LEG_SLICE[leg_name], ctrl_i] = result.tau
            foot_traj.pos_des[LEG_SLICE[leg_name], ctrl_i] = result.pos_des
            foot_traj.pos_now[LEG_SLICE[leg_name], ctrl_i] = result.pos_now
            foot_traj.vel_des[LEG_SLICE[leg_name], ctrl_i] = result.vel_des
            foot_traj.vel_now[LEG_SLICE[leg_name], ctrl_i] = result.vel_now

        tau_cmd[:, ctrl_i] = np.clip(tau_raw[:, ctrl_i], -TAU_LIM, TAU_LIM)
        tau_hold = tau_cmd[:, ctrl_i].copy()
        tau_log_ctrl_Nm[ctrl_i, :] = tau_hold

        ctrl_i += 1

    mj.mj_step1(mujoco_go2.model, mujoco_go2.data)
    mujoco_go2.set_joint_torque(tau_hold)
    mj.mj_step2(mujoco_go2.model, mujoco_go2.data)

    t_after = float(mujoco_go2.data.time)
    if t_after + 1e-12 >= next_render_t:
        time_log_render.append(t_after)
        q_log_render.append(mujoco_go2.data.qpos.copy())
        tau_log_render.append(tau_hold.copy())
        next_render_t += RENDER_DT

sim_end_time = time.perf_counter()
print(
    f"\nSimulation ended."
    f"\nElapsed time: {sim_end_time - sim_start_time:.3f}s"
    f"\nControl ticks: {ctrl_i}/{CTRL_STEPS}"
)

# ---------------------------------------------------------------------------
# PID-DE-MPC Statistics
# ---------------------------------------------------------------------------
total_mpc_calls = ctrl_i // STEPS_PER_MPC
n_solves = mpc.mpc_solve_count
n_reuses = mpc.mpc_reuse_count
print(f"\n=== PID-DE-MPC Statistics ===")
print(f"Total MPC update opportunities:  {total_mpc_calls}")
print(f"Actual QP solves:                {n_solves}")
print(f"Solution reuses:                 {n_reuses}")
if total_mpc_calls > 0:
    print(f"Solve rate:                      {n_solves / total_mpc_calls * 100:.1f}%")
    print(f"Reuse rate:                      {n_reuses / total_mpc_calls * 100:.1f}%")

# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
t_vec = np.arange(ctrl_i) * CTRL_DT
plot_swing_foot_traj(t_vec, foot_traj, label=LABEL)
plot_mpc_result(t_vec, mpc_force_world, tau_cmd, x_vec, label=LABEL)
plot_solve_time(mpc_solve_time_ms, mpc_update_time_ms, MPC_DT, MPC_HZ, label=LABEL)

time_log_render = np.asarray(time_log_render, dtype=float)
q_log_render = np.asarray(q_log_render, dtype=float)
tau_log_render = np.asarray(tau_log_render, dtype=float)

mujoco_go2.replay_simulation(
    time_log_render, q_log_render, tau_log_render, RENDER_DT, REALTIME_FACTOR
)
hold_until_all_fig_closed(label=LABEL)
