import numpy as np
import matplotlib.pyplot as plt
from mujoco_model import MuJoCo_GO2_Model
import mujoco as mj

def pd_controller_static(mujoco_go2, leg: str):

    q  = mujoco_go2.get_leg_joint_pos(leg)
    v = mujoco_go2.get_leg_joint_vel(leg)

    # Lazily create and store hold targets on the wrapper
    if not hasattr(mujoco_go2, "_hold_targets"):
        mujoco_go2._hold_targets = {}
    if leg not in mujoco_go2._hold_targets:
        mujoco_go2._hold_targets[leg] = q.copy()

    q_des = mujoco_go2._hold_targets[leg]
    Kp = np.array([40.0, 60.0, 40.0])
    Kd = np.array([2.5, 3.0, 2.0])

    # PD torque (positive toward q_des)
    tau_pd = Kp * (q_des - q) - Kd * v

    # Add MuJoCo's bias forces (gravity, Coriolis, etc.) for the leg's DoFs
    # Assumes the wrapper exposes model/data as .model / .data
    m, d = mujoco_go2.model, mujoco_go2.data
    jnames = [f"{leg}_hip_joint", f"{leg}_thigh_joint", f"{leg}_calf_joint"]
    tau_bias = np.zeros(3)
    for i, jn in enumerate(jnames):
        jid  = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, jn)
        didx = m.jnt_dofadr[jid]               # DoF index for this joint
        tau_bias[i] = d.qfrc_bias[didx]        # MuJoCo bias torque at this DoF
    tau = tau_pd + tau_bias

    return tau
    

def make_swing_trajectory(p0, pf, t_swing, h_sw=0.0):

    p0 = np.asarray(p0, dtype=float)
    pf = np.asarray(pf, dtype=float)
    T = float(t_swing)
    dp = pf - p0

    def eval_at(t):
        # phase s in [0,1]
        s = np.clip(t / T, 0.0, 1.0)

        # Minimum-jerk basis and its derivatives
        mj   = 10*s**3 - 15*s**4 + 6*s**5
        dmj  = 30*s**2 - 60*s**3 + 30*s**4           # d(mj)/ds
        d2mj = 60*s    - 180*s**2 + 120*s**3         # d^2(mj)/ds^2

        # Base (x,y,z) trajectory
        p = p0 + dp * mj
        v = (dp * dmj) / T
        a = (dp * d2mj) / (T**2)

        # Optional smooth z-bump: b(s)=64*s^3*(1-s)^3, with zero vel/acc at ends
        if h_sw != 0.0:
            b    = 64 * s**3 * (1 - s)**3
            db   = 192 * s**2 * (1 - s)**2 * (1 - 2*s)           # db/ds
            d2b  = 192 * ( 2*s*(1 - s)**2*(1 - 2*s)
                           - 2*s**2*(1 - s)*(1 - 2*s)
                           - 2*s**2*(1 - s)**2 )                  # d^2b/ds^2

            p[2] += h_sw * b
            v[2] += h_sw * db / T
            a[2] += h_sw * d2b / (T**2)

        return p, v, a

    return eval_at



traj = make_swing_trajectory([0,0,0], [0.5, 0, 0], t_swing=1, h_sw=0.1)
p, v, a = traj(0.15)  # evaluate at mid-swing


# sample the path
T = 1.0
N = 300
ts = np.linspace(0.0, T, N)
P = np.zeros((N, 3))
for i, t in enumerate(ts):
    p, v, a = traj(t)
    P[i] = p

# 3D plot
fig = plt.figure(figsize=(7,6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(P[:,0], P[:,1], P[:,2])
ax.scatter([P[0,0]], [P[0,1]], [P[0,2]], s=40, marker='o', label='start')
ax.scatter([P[-1,0]], [P[-1,1]], [P[-1,2]], s=50, marker='^', label='end')
ax.set_xlabel('x (m)')
ax.set_ylabel('y (m)')
ax.set_zlabel('z (m)')
ax.set_title('Swing-Foot Trajectory (min-jerk + C² z-bump)')
ax.legend()
plt.show()
