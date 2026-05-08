"""
Demo 00: Command-scheduled walking + turning (mixed body velocity commands)

  0-1.0 s   : walk forward at 0.7 m/s
  1.0-1.5 s : come to a stop (zero velocity)
  1.5-3.0 s : sidestep at 0.3 m/s
  3.0-4.0 s : stop
  4.0-6.0 s : turn in place at 2.0 rad/s
  6.0-6.5 s : stop
  6.5-8.0 s : walk forward (0.6 m/s) while turning (2.0 rad/s)
  8.0-9.0 s : speed up forward to 0.8 m/s (no turning)
  9.0-10 s  : stop

All phases maintain a constant base height reference of 0.27 m. This demo is
meant to showcase smooth transitions between different motion primitives
(forward, lateral, in-place yaw, and combined walk+turn) using a single
piecewise-constant command schedule.
"""

import os


os.environ["MPLBACKEND"] = "Agg"
import time
from dataclasses import dataclass, field

import mujoco as mj
import numpy as np

from convex_mpc.centroidal_mpc import CentroidalMPC
from convex_mpc.com_trajectory import ComTraj
from convex_mpc.gait import Gait
from convex_mpc.go2_robot_data import PinGo2Model
from convex_mpc.leg_controller import LegController
from convex_mpc.mujoco_model import MuJoCo_GO2_Model
from convex_mpc.plot_helper import (
    hold_until_all_fig_closed,
    plot_mpc_result,
    plot_solve_time,
    plot_swing_foot_traj,
)


# --------------------------------------------------------------------------------
# Parameters
# --------------------------------------------------------------------------------

# Simulation Setting
INITIAL_X_POS = -5
INITIAL_Y_POS = 0
RUN_SIM_LENGTH_S = 10.0

RENDER_HZ = 120.0
RENDER_DT = 1.0 / RENDER_HZ
REALTIME_FACTOR = 1


# Locomotion Command
@dataclass
class BodyCmdPhase:
    t_start: float
    t_end: float
    x_vel: float
    y_vel: float
    z_pos: float
    yaw_rate: float


# Command format:
#   [start_time (s), end_time (s), x_velocity (m/s), y_velocity (m/s),
#    z_position (m), yaw_angular_velocity (rad/s)]
CMD_SCHEDULE = [
    BodyCmdPhase(0.0, 1.0, 0.7, 0.0, 0.27, 0.0),  # Forward 0.7 m/s
    BodyCmdPhase(1.0, 1.5, 0.0, 0.0, 0.27, 0.0),  # Stop
    BodyCmdPhase(1.5, 3.0, 0.0, 0.3, 0.27, 0.0),  # Sideway 0.3 m/s
    BodyCmdPhase(3.0, 4.0, 0.0, 0.0, 0.27, 0.0),  # Stop
    BodyCmdPhase(4.0, 6.0, 0.0, 0.0, 0.27, 2.0),  # Rotate 2 rad/s
    BodyCmdPhase(6.0, 6.5, 0.0, 0.0, 0.27, 0.0),  # Stop
    BodyCmdPhase(6.5, 8.0, 0.6, 0.0, 0.27, 2.0),  # Forward 0.6 m/s + Rotate 2 rad/s
    BodyCmdPhase(8.0, 9.0, 0.8, 0.0, 0.27, 0.0),  # Forward 0.8 m/s
    BodyCmdPhase(9.0, 10.0, 0.0, 0.0, 0.27, 0.0),  # Stop
]

# Gait Setting
GAIT_HZ = 3
GAIT_DUTY = 0.6
GAIT_T = 1.0 / GAIT_HZ

# Trajectory Reference Setting (defaults)
x_vel_des_body = 0.0
y_vel_des_body = 0.0
z_pos_des_body = 0.27
yaw_rate_des_body = 0.0

# MuJoCo Sim Update Rate
SIM_HZ = 1000
SIM_DT = 1.0 / SIM_HZ

# Leg Coontroller Update Rate
CTRL_HZ = 200  # 200 Hz
CTRL_DT = 1.0 / CTRL_HZ

# Must be an integer ratio for clean decimation
if SIM_HZ % CTRL_HZ != 0:
    raise ValueError(
        f"SIM_HZ ({SIM_HZ}) must be divisible by CTRL_HZ ({CTRL_HZ}) for this decimation method."
    )
CTRL_DECIM = SIM_HZ // CTRL_HZ

SIM_STEPS = int(RUN_SIM_LENGTH_S * SIM_HZ)
CTRL_STEPS = int(RUN_SIM_LENGTH_S * CTRL_HZ)

# Relation between MPC loop and control loop
MPC_DT = GAIT_T / 16
MPC_HZ = 1.0 / MPC_DT
STEPS_PER_MPC = max(1, int(CTRL_HZ // MPC_HZ))  # MPC update every N control ticks

# Go2 Joint Torque Limit
HIP_LIM = 23.7
ABD_LIM = 23.7
KNEE_LIM = 45.43
SAFETY = 0.9

TAU_LIM = SAFETY * np.array(
    [
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,  # FL: hip, thigh, calf
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,  # FR
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,  # RL
        HIP_LIM,
        ABD_LIM,
        KNEE_LIM,  # RR
    ]
)

LEG_SLICE = {
    "FL": slice(0, 3),
    "FR": slice(3, 6),
    "RL": slice(6, 9),
    "RR": slice(9, 12),
}


# --------------------------------------------------------------------------------
# Helper Function
# --------------------------------------------------------------------------------
def get_body_cmd(t: float):
    for phase in CMD_SCHEDULE:
        if phase.t_start <= t < phase.t_end:
            return phase.x_vel, phase.y_vel, phase.z_pos, phase.yaw_rate
    return 0.0, 0.0, 0.27, 0.0


# --------------------------------------------------------------------------------
# Storage Variables (CONTROL-rate logs for plots)
# --------------------------------------------------------------------------------

# Centroidal state x = [px, py, pz, r, p, y, vx, vy, vz, wx, wy, wz]
x_vec = np.zeros((12, CTRL_STEPS))

# MPC contact force log (world): [FLx,FLy,FLz, FRx,FRy,FRz, RLx,RLy,RLz, RRx,RRy,RRz]
mpc_force_world = np.zeros((12, CTRL_STEPS))

# Torques
tau_raw = np.zeros((12, CTRL_STEPS))
tau_cmd = np.zeros((12, CTRL_STEPS))

# Control-rate log (if you want it)
time_log_ctrl_s = np.zeros(CTRL_STEPS)
q_log_ctrl = np.zeros((CTRL_STEPS, 19))
tau_log_ctrl_Nm = np.zeros((CTRL_STEPS, 12))


# Foot trajectory logs (control-rate)
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

# --------------------------------------------------------------------------------
# Simulation Initialization
# --------------------------------------------------------------------------------

go2 = PinGo2Model()
mujoco_go2 = MuJoCo_GO2_Model()
leg_controller = LegController()
traj = ComTraj(go2)
gait = Gait(GAIT_HZ, GAIT_DUTY)

# Initialize robot configuration
q_init = go2.current_config.get_q()
q_init[0], q_init[1] = INITIAL_X_POS, INITIAL_Y_POS
mujoco_go2.update_with_q_pin(q_init)

# Set physics dt (keep it fast!)
mujoco_go2.model.opt.timestep = SIM_DT

# Initialize MPC
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
mpc = CentroidalMPC(go2, traj)

# Safe defaults until first solve
U_opt = np.zeros((12, traj.N), dtype=float)

# --------------------------------------------------------------------------------
# Replay logs sampled at RENDER_HZ
# --------------------------------------------------------------------------------
time_log_render = []
q_log_render = []
tau_log_render = []

next_render_t = 0.0

# --------------------------------------------------------------------------------
# Simulation Loop
# --------------------------------------------------------------------------------
print(f"Running simulation for {RUN_SIM_LENGTH_S}s")
sim_start_time = time.perf_counter()

ctrl_i = 0
tau_hold = np.zeros(12, dtype=float)

for k in range(SIM_STEPS):
    time_now_s = float(mujoco_go2.data.time)

    # Control update at CTRL_HZ
    if (k % CTRL_DECIM) == 0 and ctrl_i < CTRL_STEPS:
        # Commands (updated at control rate)
        x_vel_des_body, y_vel_des_body, z_pos_des_body, yaw_rate_des_body = get_body_cmd(time_now_s)

        # Update Pinocchio from current MuJoCo state
        mujoco_go2.update_pin_with_mujoco(go2)

        x_vec[:, ctrl_i] = go2.compute_com_x_vec().reshape(-1)

        # Control-rate logs
        time_log_ctrl_s[ctrl_i] = time_now_s
        q_log_ctrl[ctrl_i, :] = mujoco_go2.data.qpos

        # Update MPC if needed
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
            X_opt = w_opt[: 12 * (N)].reshape((12, N), order="F")
            U_opt = w_opt[12 * (N) :].reshape((12, N), order="F")

        # Extract first GRF from MPC
        mpc_force_world[:, ctrl_i] = U_opt[:, 0]

        # Compute joint torques
        FL = leg_controller.compute_leg_torque(
            "FL", go2, gait, mpc_force_world[LEG_SLICE["FL"], ctrl_i], time_now_s
        )
        tau_raw[LEG_SLICE["FL"], ctrl_i] = FL.tau
        foot_traj.pos_des[LEG_SLICE["FL"], ctrl_i] = FL.pos_des
        foot_traj.pos_now[LEG_SLICE["FL"], ctrl_i] = FL.pos_now
        foot_traj.vel_des[LEG_SLICE["FL"], ctrl_i] = FL.vel_des
        foot_traj.vel_now[LEG_SLICE["FL"], ctrl_i] = FL.vel_now

        FR = leg_controller.compute_leg_torque(
            "FR", go2, gait, mpc_force_world[LEG_SLICE["FR"], ctrl_i], time_now_s
        )
        tau_raw[LEG_SLICE["FR"], ctrl_i] = FR.tau
        foot_traj.pos_des[LEG_SLICE["FR"], ctrl_i] = FR.pos_des
        foot_traj.pos_now[LEG_SLICE["FR"], ctrl_i] = FR.pos_now
        foot_traj.vel_des[LEG_SLICE["FR"], ctrl_i] = FR.vel_des
        foot_traj.vel_now[LEG_SLICE["FR"], ctrl_i] = FR.vel_now

        RL = leg_controller.compute_leg_torque(
            "RL", go2, gait, mpc_force_world[LEG_SLICE["RL"], ctrl_i], time_now_s
        )
        tau_raw[LEG_SLICE["RL"], ctrl_i] = RL.tau
        foot_traj.pos_des[LEG_SLICE["RL"], ctrl_i] = RL.pos_des
        foot_traj.pos_now[LEG_SLICE["RL"], ctrl_i] = RL.pos_now
        foot_traj.vel_des[LEG_SLICE["RL"], ctrl_i] = RL.vel_des
        foot_traj.vel_now[LEG_SLICE["RL"], ctrl_i] = RL.vel_now

        RR = leg_controller.compute_leg_torque(
            "RR", go2, gait, mpc_force_world[LEG_SLICE["RR"], ctrl_i], time_now_s
        )
        tau_raw[LEG_SLICE["RR"], ctrl_i] = RR.tau
        foot_traj.pos_des[LEG_SLICE["RR"], ctrl_i] = RR.pos_des
        foot_traj.pos_now[LEG_SLICE["RR"], ctrl_i] = RR.pos_now
        foot_traj.vel_des[LEG_SLICE["RR"], ctrl_i] = RR.vel_des
        foot_traj.vel_now[LEG_SLICE["RR"], ctrl_i] = RR.vel_now

        # Saturate + hold
        tau_cmd[:, ctrl_i] = np.clip(tau_raw[:, ctrl_i], -TAU_LIM, TAU_LIM)
        tau_hold = tau_cmd[:, ctrl_i].copy()

        tau_log_ctrl_Nm[ctrl_i, :] = tau_hold

        ctrl_i += 1

    # Apply held torques at every SIM step
    mj.mj_step1(mujoco_go2.model, mujoco_go2.data)
    mujoco_go2.set_joint_torque(tau_hold)
    mj.mj_step2(mujoco_go2.model, mujoco_go2.data)

    # Render-rate logging for smooth replay
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

# --------------------------------------------------------------------------------
# Simulation Results
# --------------------------------------------------------------------------------

# Plot results
t_vec = np.arange(ctrl_i) * CTRL_DT
plot_swing_foot_traj(t_vec, foot_traj, False)
plot_mpc_result(t_vec, mpc_force_world, tau_cmd, x_vec, block=False)
plot_solve_time(mpc_solve_time_ms, mpc_update_time_ms, MPC_DT, MPC_HZ, block=True)

# Replay simulation
time_log_render = np.asarray(time_log_render, dtype=float)
q_log_render = np.asarray(q_log_render, dtype=float)
tau_log_render = np.asarray(tau_log_render, dtype=float)

mujoco_go2.replay_simulation(
    time_log_render, q_log_render, tau_log_render, RENDER_DT, REALTIME_FACTOR
)
hold_until_all_fig_closed()
