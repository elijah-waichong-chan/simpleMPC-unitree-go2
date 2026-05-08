"""
Demo 11: Compare EMPC, DEMPC, PID-DE-MPC vs CentroidalMPC — number of QP rollouts.

Runs the same trot-forward scenario for all four controllers:
  1. CentroidalMPC   (baseline — re-solves QP every MPC step)
  2. EMPC            (static event-trigger, no DOB)
  3. DEMPC           (static event-trigger + DOB compensation)
  4. PID-DE-MPC      (PID event-trigger + DOB compensation)

Compares total QP rollouts and plots the result.
"""

import os


os.environ["MPLBACKEND"] = "Agg"
from dataclasses import dataclass

import mujoco as mj
import numpy as np

from convex_mpc.centroidal_mpc import CentroidalMPC
from convex_mpc.com_trajectory import ComTraj
from convex_mpc.dempc import CentroidalDEMPC
from convex_mpc.empc import CentroidalEMPC
from convex_mpc.gait import Gait
from convex_mpc.go2_robot_data import PinGo2Model
from convex_mpc.leg_controller import LegController
from convex_mpc.mujoco_model import MuJoCo_GO2_Model
from convex_mpc.pid_de_mpc import CentroidalPIDDEMPC


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
INITIAL_X_POS = -5
INITIAL_Y_POS = 0
RUN_SIM_LENGTH_S = 10.0

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
    BodyCmdPhase(0.0, 10.0, 0.5, 0.0, 0.27, 0.0),
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


def get_body_cmd(t: float):
    for phase in CMD_SCHEDULE:
        if phase.t_start <= t < phase.t_end:
            return phase.x_vel, phase.y_vel, phase.z_pos, phase.yaw_rate
    return 0.0, 0.0, 0.27, 0.0


def run_simulation(controller_type: str):
    """Run a simulation with the given controller and return solve-count + trajectory stats."""

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

    # Select controller
    if controller_type == "MPC":
        mpc = CentroidalMPC(go2, traj)
    elif controller_type == "EMPC":
        mpc = CentroidalEMPC(go2, traj, static_threshold=0.8)
    elif controller_type == "DEMPC":
        mpc = CentroidalDEMPC(go2, traj, mpc_dt=MPC_DT, static_threshold=0.8)
    elif controller_type == "PID-DE-MPC":
        mpc = CentroidalPIDDEMPC(go2, traj, mpc_dt=MPC_DT)
    else:
        raise ValueError(f"Unknown controller: {controller_type}")

    U_opt = np.zeros((12, traj.N), dtype=float)

    ctrl_i = 0
    tau_hold = np.zeros(12, dtype=float)
    solve_log = []
    mpc_step_idx = 0

    # Trajectory logs (sampled at MPC rate)
    max_mpc_steps = CTRL_STEPS // STEPS_PER_MPC + 1
    x_log = np.zeros((12, max_mpc_steps))
    u_log = np.zeros((12, max_mpc_steps))
    t_log = np.zeros(max_mpc_steps)

    for k in range(SIM_STEPS):
        time_now_s = float(mujoco_go2.data.time)

        if (k % CTRL_DECIM) == 0 and ctrl_i < CTRL_STEPS:
            cmd_x_vel, cmd_y_vel, cmd_z_pos, cmd_yaw_rate = get_body_cmd(time_now_s)
            mujoco_go2.update_pin_with_mujoco(go2)

            if (ctrl_i % STEPS_PER_MPC) == 0:
                traj.generate_traj(
                    go2,
                    gait,
                    time_now_s,
                    cmd_x_vel,
                    cmd_y_vel,
                    cmd_z_pos,
                    cmd_yaw_rate,
                    time_step=MPC_DT,
                )

                sol = mpc.solve_QP(go2, traj, False)

                N = traj.N
                w_opt = sol["x"].full().flatten()
                U_opt = w_opt[12 * N :].reshape((12, N), order="F")

                # Log state and control at this MPC step
                if mpc_step_idx < max_mpc_steps:
                    x_log[:, mpc_step_idx] = go2.compute_com_x_vec().flatten()
                    u_log[:, mpc_step_idx] = U_opt[:, 0]
                    t_log[mpc_step_idx] = time_now_s

                if controller_type == "MPC":
                    did_solve = True
                else:
                    did_solve = mpc.solve_time > 0.0
                solve_log.append((mpc_step_idx, did_solve))
                mpc_step_idx += 1

            mpc_force_world = U_opt[:, 0]

            for leg_name in ["FL", "FR", "RL", "RR"]:
                result = leg_controller.compute_leg_torque(
                    leg_name, go2, gait, mpc_force_world[LEG_SLICE[leg_name]], time_now_s
                )
                tau_raw = result.tau
                tau_cmd = np.clip(tau_raw, -TAU_LIM[:3], TAU_LIM[:3])
                tau_hold[LEG_SLICE[leg_name]] = tau_cmd

            ctrl_i += 1

        mj.mj_step1(mujoco_go2.model, mujoco_go2.data)
        mujoco_go2.set_joint_torque(tau_hold)
        mj.mj_step2(mujoco_go2.model, mujoco_go2.data)

    # Trim logs to actual length
    x_log = x_log[:, :mpc_step_idx]
    u_log = u_log[:, :mpc_step_idx]
    t_log = t_log[:mpc_step_idx]

    total_opportunities = mpc_step_idx
    n_solves = sum(1 for _, did in solve_log if did)

    print(
        f"  {controller_type:12s}: {n_solves:4d} / {total_opportunities} QP solves "
        f"({n_solves / total_opportunities * 100:.1f}%)"
    )

    return {
        "label": controller_type,
        "total_opportunities": total_opportunities,
        "n_solves": n_solves,
        "solve_log": solve_log,
        "x_log": x_log,
        "u_log": u_log,
        "t_log": t_log,
    }


# ---------------------------------------------------------------------------
# Run all four simulations
# ---------------------------------------------------------------------------
print("Comparing MPC rollout counts (10 s trot forward @ 0.5 m/s)\n")

controllers = ["MPC", "EMPC", "DEMPC", "PID-DE-MPC"]
results = {}

for ctrl in controllers:
    print(f"Running {ctrl} ...")
    results[ctrl] = run_simulation(ctrl)

# ---------------------------------------------------------------------------
# Comparison plot
# ---------------------------------------------------------------------------
import matplotlib.pyplot as plt


fig, axes = plt.subplots(2, 2, figsize=(16, 10))

# --- Top-left: Bar chart of total solves ---
ax = axes[0, 0]
labels = [r["label"] for r in results.values()]
solves = [r["n_solves"] for r in results.values()]
total = list(results.values())[0]["total_opportunities"]
colors = ["gray", "darkorange", "seagreen", "steelblue"]

bars = ax.bar(labels, solves, color=colors, width=0.5)
ax.set_ylabel("Number of QP solves")
ax.set_title(f"Total QP Solves (opportunities = {total})")
ax.grid(True, alpha=0.3, axis="y")

for bar, val in zip(bars, solves):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 1,
        f"{val}\n({val / total * 100:.1f}%)",
        ha="center",
        va="bottom",
        fontweight="bold",
        fontsize=11,
    )

# Baseline reference line
ax.axhline(y=total, color="gray", linestyle="--", linewidth=1, alpha=0.5)
ax.text(0.02, total + 1, f"MPC baseline ({total})", fontsize=9, color="gray")

# --- Top-right: Event-trigger timelines ---
ax = axes[0, 1]
event_controllers = ["EMPC", "DEMPC", "PID-DE-MPC"]
event_colors = ["darkorange", "seagreen", "steelblue"]

for offset, (ctrl, color) in enumerate(zip(event_controllers, event_colors)):
    log = results[ctrl]["solve_log"]
    steps = [s for s, _ in log]
    did_solve = [did for _, did in log]
    y_pos = [offset + 0.5] * len(steps)
    face_colors = [color if d else "lightgray" for d in did_solve]
    ax.barh(
        y_pos,
        [1] * len(steps),
        left=steps,
        height=0.4,
        color=face_colors,
        edgecolor="white",
        linewidth=0.5,
    )

ax.set_yticks([0.5, 1.5, 2.5])
ax.set_yticklabels(event_controllers)
ax.set_xlabel("MPC step index")
ax.set_title("Event-Trigger Timeline (blue/green/orange = solve, gray = reuse)")

# --- Bottom-left: Solve rate comparison ---
ax = axes[1, 0]
rates = [r["n_solves"] / total * 100 for r in results.values()]
bars = ax.bar(labels, rates, color=colors, width=0.5)
ax.set_ylabel("Solve rate (%)")
ax.set_title("QP Solve Rate Comparison")
ax.grid(True, alpha=0.3, axis="y")
ax.axhline(y=100, color="gray", linestyle="--", linewidth=1, alpha=0.5)

for bar, rate in zip(bars, rates):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 1,
        f"{rate:.1f}%",
        ha="center",
        va="bottom",
        fontweight="bold",
        fontsize=11,
    )

# --- Bottom-right: Reduction vs baseline ---
ax = axes[1, 1]
baseline = results["MPC"]["n_solves"]
reductions = [(1 - r["n_solves"] / baseline) * 100 for r in results.values()]
reductions[0] = 0  # MPC baseline

bars = ax.bar(labels, reductions, color=colors, width=0.5)
ax.set_ylabel("Reduction vs MPC baseline (%)")
ax.set_title("QP Solve Reduction Relative to Baseline")
ax.grid(True, alpha=0.3, axis="y")

for bar, red in zip(bars, reductions):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 0.5,
        f"{red:.1f}%",
        ha="center",
        va="bottom",
        fontweight="bold",
        fontsize=11,
    )

fig.suptitle(
    "EMPC vs DEMPC vs PID-DE-MPC vs CentroidalMPC: QP Rollout Reduction",
    fontweight="bold",
    fontsize=14,
)
plt.tight_layout()

os.makedirs("assets", exist_ok=True)
plt.savefig("assets/ex11_mpc_rollout_compare.png", dpi=300)
plt.close(fig)

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print(f"\n{'='*60}")
print(f"{'Controller':<14} {'Solves':>6} {'Rate':>8} {'Reduction':>10}")
print(f"{'-'*60}")
for r in results.values():
    rate = r["n_solves"] / r["total_opportunities"] * 100
    red = (1 - r["n_solves"] / baseline) * 100
    print(f"{r['label']:<14} {r['n_solves']:>6d} {rate:>7.1f}% {red:>9.1f}%")
print("=" * 60)
print("Saved comparison plot to assets/ex11_mpc_rollout_compare.png")

# ---------------------------------------------------------------------------
# Figure 2: State trajectory comparison (key states, all controllers)
# ---------------------------------------------------------------------------
ctrl_colors = {"MPC": "gray", "EMPC": "darkorange", "DEMPC": "seagreen", "PID-DE-MPC": "steelblue"}
state_labels = [
    ("px", 0, "CoM X [m]"),
    ("py", 1, "CoM Y [m]"),
    ("pz", 2, "CoM Z [m]"),
    ("vx", 6, "Vel X [m/s]"),
    ("vy", 7, "Vel Y [m/s]"),
    ("roll", 3, "Roll [rad]"),
    ("pitch", 4, "Pitch [rad]"),
    ("yaw_rate", 11, "Yaw rate [rad/s]"),
]

fig2, axes2 = plt.subplots(4, 2, figsize=(16, 12))
for ax, (name, idx, ylabel) in zip(axes2.flat, state_labels):
    for ctrl in controllers:
        r = results[ctrl]
        ax.plot(
            r["t_log"],
            r["x_log"][idx, :],
            color=ctrl_colors[ctrl],
            linewidth=1.0,
            alpha=0.85,
            label=ctrl,
        )
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    if name in ("px", "roll"):
        ax.legend(fontsize=7, loc="best")

fig2.suptitle("State Trajectory Comparison Across Controllers", fontweight="bold", fontsize=14)
plt.tight_layout()
plt.savefig("assets/ex11_state_comparison.png", dpi=300)
plt.close(fig2)

# ---------------------------------------------------------------------------
# Figure 3: Control force comparison (fz per leg, all controllers)
# ---------------------------------------------------------------------------
leg_labels = [("FL fz", 2), ("FR fz", 5), ("RL fz", 8), ("RR fz", 11)]

fig3, axes3 = plt.subplots(2, 2, figsize=(16, 8))
for ax, (leg_name, idx) in zip(axes3.flat, leg_labels):
    for ctrl in controllers:
        r = results[ctrl]
        ax.plot(
            r["t_log"],
            r["u_log"][idx, :],
            color=ctrl_colors[ctrl],
            linewidth=1.0,
            alpha=0.85,
            label=ctrl,
        )
    ax.set_ylabel(f"{leg_name} [N]")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)
    if leg_name == "FL fz":
        ax.legend(fontsize=7, loc="best")

fig3.suptitle("Contact Force (fz) Comparison Across Controllers", fontweight="bold", fontsize=14)
plt.tight_layout()
plt.savefig("assets/ex11_control_comparison.png", dpi=300)
plt.close(fig3)

print("Saved state comparison to assets/ex11_state_comparison.png")
print("Saved control comparison to assets/ex11_control_comparison.png")
