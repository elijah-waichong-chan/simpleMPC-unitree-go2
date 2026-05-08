"""
Example 05: PID-DE-MPC — MATLAB replication.

Replicates the PID-based Dual-mode Event-triggered MPC with Disturbance
Observer (ESO) from piddempc.m for a 2D nonlinear system:

    x1_dot = x2
    x2_dot = -tao/M * exp(-x1) * x1 - hd/M * x2 + u/M + w(t)/M

with external disturbance w(t) = 0.1*sin(t*1.4).
"""

import matplotlib.pyplot as plt
import numpy as np

from convex_mpc.pid_de_mpc import PIDDEMPC


# ---------------------------------------------------------------------------
# Parameters (matching piddempc.m)
# ---------------------------------------------------------------------------
tao = 0.9
M = 1.25
hd = 0.42
Ts = 0.1
Tp = 10.0
N_steps = int(Tp / Ts)

T_horizon = int(2.5 / Ts)  # MPC prediction horizon

Q = 0.2 * np.eye(2)
R = 0.1
P = np.array([[0.1692, 0.0572], [0.0572, 0.1391]])
K = np.array([-0.4454, -1.0932])

w0 = 12
Kp = 0.14
Ki = 0.9
Kd = 0.7
r_trigger = 0.001508
pid_threshold = 0.01

x0 = np.array([1.2, -2.0])

# ---------------------------------------------------------------------------
# External disturbance w(t)
# ---------------------------------------------------------------------------
t_vec = np.arange(N_steps) * Ts
w = 0.1 * np.sin(t_vec * 1.4)


# ---------------------------------------------------------------------------
# Nonlinear drift function
# ---------------------------------------------------------------------------
def f_dyn(x1, x2):
    return -tao / M * np.exp(-x1) * x1 - hd / M * x2


# ---------------------------------------------------------------------------
# Initialise controller
# ---------------------------------------------------------------------------
controller = PIDDEMPC(
    tao=tao,
    M=M,
    hd=hd,
    Ts=Ts,
    T=T_horizon,
    Q=Q,
    R=R,
    P=P,
    K=K,
    w0=w0,
    Kp=Kp,
    Ki=Ki,
    Kd=Kd,
    r_trigger=r_trigger,
    pid_threshold=pid_threshold,
    u_min=-1.0,
    u_max=1.0,
)
controller.reset(x0)

# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------
xc = np.zeros((2, N_steps))
uc = np.zeros(N_steps)
deso_log = np.zeros(N_steps)
mode_log = [""] * N_steps

xc[:, 0] = x0

for i in range(N_steps - 1):
    # Log pre-update ESO value (matches MATLAB: deso(i) plotted at time i)
    deso_log[i] = controller.deso

    # Compute control (u = u_mpc - udo)
    u_val, info = controller.compute_control(xc[:, i], i)
    uc[i] = u_val
    mode_log[i] = info["mode"]

    # Apply to plant (with external disturbance w)
    x1_cur, x2_cur = xc[0, i], xc[1, i]
    xc[0, i + 1] = x1_cur + Ts * x2_cur
    xc[1, i + 1] = x2_cur + Ts * (f_dyn(x1_cur, x2_cur) + u_val / M + w[i] / M)

    # ESO update uses state BEFORE control (matches MATLAB lines 155-157)
    controller.update_eso(xc[:, i], u_val)

# Log final pre-update value for the last step
deso_log[-1] = controller.deso
mode_log[-1] = mode_log[-2] if N_steps > 1 else ""

# Event times (indices where MPC was re-solved)
et = np.array(controller.event_times)

# ---------------------------------------------------------------------------
# Plots (matching piddempc.m figures)
# ---------------------------------------------------------------------------
t_plot = np.arange(N_steps) * Ts

# Figure 1: State trajectories
plt.figure(1, figsize=(8, 4))
plt.plot(t_plot, xc[0, :], "k", linewidth=1.2, label=r"$x_1$")
plt.plot(t_plot, xc[1, :], "r", linewidth=1.2, label=r"$x_2$")
plt.xlabel("Time [s]")
plt.ylabel("State")
plt.title("State Trajectories")
plt.legend()
plt.grid(True, alpha=0.3)

# Figure 2: Control input
plt.figure(2, figsize=(8, 4))
plt.plot(t_plot, uc, "k", linewidth=1.2)
plt.xlabel("Time [s]")
plt.ylabel("Control input $u$")
plt.title("Control Input")
plt.grid(True, alpha=0.3)
plt.ylim([-1.1, 1.1])

# Figure 3: Event times
plt.figure(3, figsize=(8, 2))
if len(et) > 0:
    et_t = np.array(et) * Ts
    for t_ev in et_t:
        plt.axvline(x=t_ev, color="b", linewidth=1.0)
plt.xlabel("Time [s]")
plt.title("MPC Re-solve Events")
plt.grid(True, alpha=0.3)
plt.yticks([])

# Figure 4: Disturbance w vs ESO estimate deso
plt.figure(4, figsize=(8, 4))
plt.plot(t_plot, w, "r", linewidth=1.2, label=r"$w(t)$ (true)")
plt.plot(t_plot, deso_log, "b", linewidth=1.2, label=r"$\hat{d}$ (ESO)")
plt.xlabel("Time [s]")
plt.ylabel("Disturbance")
plt.title("Disturbance vs ESO Estimate")
plt.legend()
plt.grid(True, alpha=0.3)

# Summary statistics
n_mpc_solves = len(controller.event_times)
n_total = N_steps
mpc_modes = sum(1 for m in mode_log if m.startswith("MPC"))
lqr_modes = n_total - mpc_modes

print(f"\n=== PID-DE-MPC Simulation Summary ===")
print(f"Total steps:            {n_total}")
print(f"MPC re-solves:          {n_mpc_solves}")
print(f"MPC-mode steps:         {mpc_modes}")
print(f"LQR-mode steps:         {lqr_modes}")
print(f"MPC solve rate:         {n_mpc_solves / n_total * 100:.1f}%")
print(f"Final state:            x1 = {xc[0, -1]:.4f}, x2 = {xc[1, -1]:.4f}")

plt.show()
