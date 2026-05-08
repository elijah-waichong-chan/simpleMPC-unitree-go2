import os

import matplotlib.pyplot as plt
import numpy as np


def _ensure_assets_dir():
    os.makedirs("assets", exist_ok=True)


def plot_contact_forces(
    U_opt, contact_mask, dt, label="mpc", leg_names=("FL", "FR", "RL", "RR"), block=False
):
    _ensure_assets_dir()
    assert U_opt.shape[0] == 12
    N = U_opt.shape[1]
    t_edges = np.linspace(0, N * dt, N + 1)

    def F(leg_idx):
        base = 3 * leg_idx
        Fleg = U_opt[base : base + 3, :]
        return Fleg[0, :], Fleg[1, :], Fleg[2, :]

    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for i, ax in enumerate(axes):
        fx, fy, fz = F(i)
        ax.stairs(fx, t_edges, label="fx")
        ax.stairs(fy, t_edges, label="fy")
        ax.stairs(fz, t_edges, label="fz", linewidth=2)
        swing = contact_mask[i] == 0
        for k in np.flatnonzero(swing):
            ax.axvspan(t_edges[k], t_edges[k + 1], alpha=0.15, hatch="//", edgecolor="none")
        ax.set_ylabel(f"{leg_names[i]} [N]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", ncols=3, fontsize=9)
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("Leg Contact Forces (one gait cycle)")
    plt.tight_layout()
    plt.savefig(f"assets/{label}_contact_forces.png", dpi=300)
    plt.close(fig)


def plot_traj_tracking(pos_traj_ref, pos_traj_sim, label="mpc", block=False):
    _ensure_assets_dir()
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pos_traj_ref[0, 0], pos_traj_ref[1, 0], pos_traj_ref[2, 0], label="Initial Position")
    ax.scatter(
        pos_traj_ref[0, :],
        pos_traj_ref[1, :],
        pos_traj_ref[2, :],
        "b--",
        linewidth=2,
        label="Reference Trajectory",
    )
    ax.plot(
        pos_traj_sim[0, :],
        pos_traj_sim[1, :],
        pos_traj_sim[2, :],
        "g-",
        linewidth=2,
        label="Optimal",
    )
    ax.set_title("3D Trajectory", fontsize=13)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_box_aspect([1, 1, 1])
    data = np.hstack([pos_traj_ref, pos_traj_sim])
    mins = data.min(axis=1)
    maxs = data.max(axis=1)
    ctr = (mins + maxs) / 2.0
    half = (maxs - mins).max() / 2.0
    r = max(half * 1.05, 1e-9)
    ax.set_xlim(ctr[0] - r, ctr[0] + r)
    ax.set_ylim(ctr[1] - r, ctr[1] + r)
    ax.set_zlim(ctr[2] - r, ctr[2] + r)
    ax.set_box_aspect([1, 1, 1])
    ax.grid(True)
    ax.legend(loc="best")
    plt.savefig(f"assets/{label}_3d_traj.png", dpi=300)
    plt.close(fig)


def plot_mpc_result(t_vec, force, tau, x_vec, block=False, label="mpc"):
    _ensure_assets_dir()
    fig, axes = plt.subplots(4, 3, figsize=(15, 10), constrained_layout=True)

    axis = axes[0, 0]
    axis.step(t_vec, force[0, :], label="x-direction force")
    axis.step(t_vec, force[1, :], label="y-direction force")
    axis.step(t_vec, force[2, :], label="z-direction force")
    axis.legend()
    axis.set_title("Front Left Foot Optimized Contact Force (N)")
    axis.grid(True)

    axis = axes[1, 0]
    axis.step(t_vec, force[3, :], label="f_x")
    axis.step(t_vec, force[4, :], label="f_y")
    axis.step(t_vec, force[5, :], label="f_z")
    axis.legend()
    axis.set_title("Front Right Foot Optimized Contact Force (N)")
    axis.grid(True)

    axis = axes[2, 0]
    axis.step(t_vec, force[6, :], label="f_x")
    axis.step(t_vec, force[7, :], label="f_y")
    axis.step(t_vec, force[8, :], label="f_z")
    axis.legend()
    axis.set_title("Rear Left Foot Optimized Contact Force (N)")
    axis.grid(True)

    axis = axes[3, 0]
    axis.step(t_vec, force[9, :], label="f_x")
    axis.step(t_vec, force[10, :], label="f_y")
    axis.step(t_vec, force[11, :], label="f_z")
    axis.legend()
    axis.set_title("Rear Right Foot Optimized Contact Force (N)")
    axis.grid(True)

    axis = axes[0, 1]
    axis.step(t_vec, tau[0, :], label="Hip joint torque (Nm)")
    axis.step(t_vec, tau[1, :], label="Thigh joint torque (Nm)")
    axis.step(t_vec, tau[2, :], label="Calf joint torque (Nm)")
    axis.legend()
    axis.set_title("Front Left Leg Joint Torque(Nm)")
    axis.grid(True)

    axis = axes[1, 1]
    axis.step(t_vec, tau[3, :], label="hip_tau")
    axis.step(t_vec, tau[4, :], label="thigh_tau")
    axis.step(t_vec, tau[5, :], label="calf_tau")
    axis.legend()
    axis.set_title("Front Right Leg Joint Torque(Nm)")
    axis.grid(True)

    axis = axes[2, 1]
    axis.step(t_vec, tau[6, :], label="hip_tau")
    axis.step(t_vec, tau[7, :], label="thigh_tau")
    axis.step(t_vec, tau[8, :], label="calf_tau")
    axis.legend()
    axis.set_title("Rear Left Leg Joint Torque(Nm)")
    axis.grid(True)

    axis = axes[3, 1]
    axis.step(t_vec, tau[9, :], label="hip_tau")
    axis.step(t_vec, tau[10, :], label="thigh_tau")
    axis.step(t_vec, tau[11, :], label="calf_tau")
    axis.legend()
    axis.set_title("Rear Right Leg Joint Torque(Nm)")
    axis.grid(True)

    axis = axes[0, 2]
    axis.step(t_vec, x_vec[0, :], label="x-position")
    axis.step(t_vec, x_vec[1, :], label="y-position")
    axis.step(t_vec, x_vec[2, :], label="z-position")
    axis.set_title("CoM Position in World Frame (m)")
    axis.legend()
    axis.grid(True)

    axis = axes[1, 2]
    axis.step(t_vec, x_vec[3, :], label="roll")
    axis.step(t_vec, x_vec[4, :], label="pitch")
    axis.step(t_vec, x_vec[5, :], label="yaw")
    axis.set_title("ZYX Euler (rad)")
    axis.legend()
    axis.grid(True)

    axis = axes[2, 2]
    axis.step(t_vec, x_vec[6, :], label="x-velocity")
    axis.step(t_vec, x_vec[7, :], label="y-velocity")
    axis.step(t_vec, x_vec[8, :], label="z-velocity")
    axis.set_title("CoM Velocity in World Frame (m/s)")
    axis.legend()
    axis.grid(True)

    axis = axes[3, 2]
    axis.step(t_vec, x_vec[9, :], label="roll rate")
    axis.step(t_vec, x_vec[10, :], label="pitch rate")
    axis.step(t_vec, x_vec[11, :], label="yaw rate")
    axis.set_title("Angular Velocity in World Frame")
    axis.legend()
    axis.grid(True)

    plt.savefig(f"assets/{label}_full_traj.png", dpi=300)
    plt.close(fig)


def plot_swing_foot_traj(t_vec, foot_traj, block=False, label="mpc"):
    _ensure_assets_dir()
    fig, axes = plt.subplots(2, 1, figsize=(15, 10), constrained_layout=True)
    plt.title("Left Front Foot Trajectory")

    axis = axes[0]
    axis.plot(t_vec, foot_traj.pos_now[6, :], color="r", label="Actual x-position")
    axis.plot(t_vec, foot_traj.pos_now[7, :], color="g", label="Actual y-position")
    axis.plot(t_vec, foot_traj.pos_now[8, :], color="b", label="Actual z-position")
    axis.plot(
        t_vec,
        foot_traj.pos_des[6, :],
        color="r",
        linestyle=":",
        linewidth=2.5,
        label="Desired x-position",
    )
    axis.plot(
        t_vec,
        foot_traj.pos_des[7, :],
        color="g",
        linestyle=":",
        linewidth=2.5,
        label="Desired y-position",
    )
    axis.plot(
        t_vec,
        foot_traj.pos_des[8, :],
        color="b",
        linestyle=":",
        linewidth=2.5,
        label="Desired z-position",
    )
    axis.legend()
    axis.grid(True)

    axis = axes[1]
    axis.plot(t_vec, foot_traj.vel_now[6, :], color="r", label="Actual x-velocity")
    axis.plot(t_vec, foot_traj.vel_now[7, :], color="g", label="Actual y-velocity")
    axis.plot(t_vec, foot_traj.vel_now[8, :], color="b", label="Actual z-velocity")
    axis.plot(t_vec, foot_traj.vel_des[6, :], color="r", linestyle=":", label="Desired x-velocity")
    axis.plot(t_vec, foot_traj.vel_des[7, :], color="g", linestyle=":", label="Desired y-velocity")
    axis.plot(t_vec, foot_traj.vel_des[8, :], color="b", linestyle=":", label="Desired z-velocity")
    axis.legend()
    axis.grid(True)

    plt.savefig(f"assets/{label}_swing_foot_traj.png", dpi=300)
    plt.close(fig)


def plot_solve_time(
    mpc_solve_time_ms, mpc_compute_time_ms, MPC_DT, MPC_HZ, block=False, label="mpc"
):
    _ensure_assets_dir()
    fig, axis = plt.subplots(figsize=(10, 6))
    mpc_solve_time_ms = np.asarray(mpc_solve_time_ms)
    mpc_compute_time_ms = np.asarray(mpc_compute_time_ms)
    total_time_ms = mpc_solve_time_ms + mpc_compute_time_ms
    avg_total_ms = np.mean(total_time_ms)
    avg_solve_ms = np.mean(mpc_solve_time_ms)
    avg_update_ms = np.mean(mpc_compute_time_ms)
    iters = np.arange(len(mpc_solve_time_ms))
    required_time_ms = MPC_DT * 1e3

    axis.set_xlabel("MPC Step", fontweight="bold")
    axis.set_ylabel("Time (ms)", fontweight="bold")
    axis.bar(iters, mpc_compute_time_ms, label="Model Update Time (ms)")
    axis.bar(iters, mpc_solve_time_ms, bottom=mpc_compute_time_ms, label="QP Solve Time (ms)")
    axis.axhline(
        required_time_ms,
        linestyle="--",
        linewidth=2.0,
        label=f"Real-Time Budget {MPC_HZ} Hz ({required_time_ms:.1f} ms)",
    )
    axis.tick_params(axis="y")
    axis.set_ylim(bottom=0)

    text_str = (
        f"Avg Model Update Time: {avg_update_ms:.2f} ms\n"
        f"Avg QP solve time:  {avg_solve_ms:.2f} ms\n"
        f"Avg MPC Cycle Time:  {avg_total_ms:.2f} ms"
    )
    axis.text(
        0.02,
        0.7,
        text_str,
        transform=axis.transAxes,
        va="center",
        ha="left",
        bbox=dict(boxstyle="round", alpha=0.3),
    )
    plt.title("MPC Iteration Stats")
    plt.tight_layout()
    plt.legend()
    plt.savefig(f"assets/{label}_solve_time.png", dpi=300)
    plt.close(fig)


def plot_full_traj(traj_ref, x_sim, block=False, label="mpc"):
    _ensure_assets_dir()
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), constrained_layout=True)
    N = len(traj_ref[0, :])
    t_vec = range(N)

    axis = axes[0, 0]
    axis.plot(t_vec, traj_ref[0, :], color="r", label="pos_x")
    axis.plot(t_vec, traj_ref[1, :], color="g", label="pos_y")
    axis.plot(t_vec, traj_ref[2, :], color="b", label="pos_z")
    axis.plot(t_vec, x_sim[0, :], color="r", linestyle=":", linewidth=2.5, label="pos_x_des")
    axis.plot(t_vec, x_sim[1, :], color="g", linestyle=":", linewidth=2.5, label="pos_y_des")
    axis.plot(t_vec, x_sim[2, :], color="b", linestyle=":", linewidth=2.5, label="pos_z_des")
    axis.legend()
    axis.grid(True)

    axis = axes[1, 0]
    axis.plot(t_vec, traj_ref[3, :], color="r", label="roll")
    axis.plot(t_vec, traj_ref[4, :], color="g", label="pitch")
    axis.plot(t_vec, traj_ref[5, :], color="b", label="yaw")
    axis.plot(t_vec, x_sim[3, :], color="r", linestyle=":", label="roll_des")
    axis.plot(t_vec, x_sim[4, :], color="g", linestyle=":", label="pitch_des")
    axis.plot(t_vec, x_sim[5, :], color="b", linestyle=":", label="yaw_des")
    axis.legend()
    axis.grid(True)

    axis = axes[0, 1]
    axis.plot(t_vec, traj_ref[6, :], color="r", label="vel_x")
    axis.plot(t_vec, traj_ref[7, :], color="g", label="vel_y")
    axis.plot(t_vec, traj_ref[8, :], color="b", label="vel_z")
    axis.plot(t_vec, x_sim[6, :], color="r", linestyle=":", linewidth=2.5, label="vel_x_des")
    axis.plot(t_vec, x_sim[7, :], color="g", linestyle=":", linewidth=2.5, label="vel_y_des")
    axis.plot(t_vec, x_sim[8, :], color="b", linestyle=":", linewidth=2.5, label="vel_z_des")
    axis.legend()
    axis.grid(True)

    axis = axes[1, 1]
    axis.plot(t_vec, traj_ref[9, :], color="r", label="roll_rate")
    axis.plot(t_vec, traj_ref[10, :], color="g", label="pitch_rate")
    axis.plot(t_vec, traj_ref[11, :], color="b", label="yaw_rate")
    axis.plot(t_vec, x_sim[9, :], color="r", linestyle=":", label="roll_rate_des")
    axis.plot(t_vec, x_sim[10, :], color="g", linestyle=":", label="pitch_rate_des")
    axis.plot(t_vec, x_sim[11, :], color="b", linestyle=":", label="yaw_rate_des")
    axis.legend()
    axis.grid(True)

    plt.savefig(f"assets/{label}_full_traj.png", dpi=300)
    plt.close(fig)


def hold_until_all_fig_closed(label="mpc"):
    _ensure_assets_dir()
    plt.savefig(f"assets/{label}_all_figs.png", dpi=300)
