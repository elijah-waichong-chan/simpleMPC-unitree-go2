import casadi as ca
import matplotlib.pyplot as plt
import numpy as np


# ========== 系统参数与模型 ==========
def f_expr(x):
    return ca.vertcat(x[1], -0.33 * ca.exp(x[0]) * x[0] - 1.1 * x[1])


def g_expr(x):
    return ca.vertcat(0, 1)  # shape (2,1)


def h_expr(x):
    return ca.vertcat(0, 1)  # shape (2,1), 扰动匹配


# 系统动力学 (用于仿真)
def system_dynamics(x, u, w):
    x1, x2 = x[0], x[1]
    dx1 = x2
    dx2 = -0.33 * np.exp(x1) * x1 - 1.1 * x2 + u + w
    return np.array([dx1, dx2])


def rk4_step(x, u, w, dt):
    k1 = system_dynamics(x, u, w)
    k2 = system_dynamics(x + 0.5 * dt * k1, u, w)
    k3 = system_dynamics(x + 0.5 * dt * k2, u, w)
    k4 = system_dynamics(x + dt * k3, u, w)
    return x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)


# 扰动信号
def disturbance(t):
    return 0.1 * np.sin(1.4 * t)


# ========== MPC 参数 ==========
Tp = 1.0  # 预测时域 (s)
N_mpc = 20  # 离散点数
dt_mpc = Tp / N_mpc
Q = np.diag([0.04, 0.0])  # 状态代价 (阶段代价: 0.04*x1^2)
R = np.array([[0.1]])  # 控制代价
Pf = np.diag([0.01, 0.0025])  # 终端代价矩阵 (V_f = x^T Pf x)

# 约束
x_min = np.array([-5.0, -5.0])
x_max = np.array([5.0, 5.0])
u_min = -3.0
u_max = 3.0

# 终端不变集 (用 V_f(x) <= 0.01 近似)
eps_terminal = 0.01

# 线性反馈增益 K (用于终端控制律, 极点配置 s^2+5s+6=0)
# A = [[0,1],[-0.33,-1.1]], B = [0,1]
# K = [-5.67, -3.9]
K = np.array([-5.67, -3.9])

# ========== DOB 参数 ==========
L_gain = np.diag([50.0, 50.0])  # 观测器增益矩阵
Gamma = 10.0

# ========== PID 触发参数 ==========
Kp = 0.14
Ki = 0.9
Kd = 0.7
delta0 = 1.2
alpha = 0.1

# ========== 构建 MPC 优化问题 (使用 CasADi) ==========
# 定义符号变量
U = ca.MX.sym("U", N_mpc)  # 控制序列
X = ca.MX.sym("X", 2, N_mpc + 1)  # 状态序列 (x0,...,x_N)

# 初始参数 (将作为 NLP 参数传入)
x0_param = ca.MX.sym("x0", 2)

# 目标函数与约束
obj = 0
g = []  # 等式约束
lbg = []
ubg = []

# 初始状态约束
g.append(X[:, 0] - x0_param)
lbg.append(np.zeros(2))
ubg.append(np.zeros(2))

for k in range(N_mpc):
    # 阶段代价
    obj += (ca.mtimes([X[:, k].T, Q, X[:, k]]) + R[0, 0] * U[k] ** 2) * dt_mpc

    # 动力学等式约束 (使用的系统模型无扰动 w_r, 因为残差由鲁棒性处理)
    x_next = X[:, k] + dt_mpc * (f_expr(X[:, k]) + g_expr(X[:, k]) * U[k])
    g.append(X[:, k + 1] - x_next)
    lbg.append(np.zeros(2))
    ubg.append(np.zeros(2))

    # 控制约束
    g.append(U[k])
    lbg.append(u_min)
    ubg.append(u_max)

    # 状态约束
    g.append(X[:, k])
    lbg.append(x_min)
    ubg.append(x_max)

# 终端代价
obj += ca.mtimes([X[:, N_mpc].T, Pf, X[:, N_mpc]])

# 终端状态约束 (不变集)
g.append(X[:, N_mpc])
lbg.append(x_min)
ubg.append(x_max)
# V_f(x_N) <= eps_terminal
terminal_cost = ca.mtimes([X[:, N_mpc].T, Pf, X[:, N_mpc]])
g.append(terminal_cost)
lbg.append(0.0)
ubg.append(eps_terminal)

# 构建 NLP
nlp = {
    "x": ca.vertcat(ca.reshape(U, -1, 1), ca.reshape(X, -1, 1)),
    "f": obj,
    "g": ca.vertcat(*g),
    "p": x0_param,
}

opts = {"ipopt.print_level": 0, "print_time": 0}
solver = ca.nlpsol("solver", "ipopt", nlp, opts)


def solve_mpc(x_current):
    """求解 MPC，返回最优控制序列 U_opt"""
    # 初始猜测
    u_guess = np.zeros(N_mpc)
    x_guess = np.tile(x_current.reshape(-1, 1), (1, N_mpc + 1))  # 简单猜测保持当前状态
    x_init = np.concatenate([u_guess, x_guess.flatten()])

    sol = solver(x0=x_init, p=x_current, lbg=np.concatenate(lbg), ubg=np.concatenate(ubg))
    u_opt = sol["x"][:N_mpc].full().flatten()
    return u_opt


# ========== 仿真初始化 ==========
dt_sim = 0.01  # 仿真步长
t_final = 10.0
x0 = np.array([2.0, 0.0])  # 初始状态 (猜测值)

x = x0.copy()
x_hat = x0.copy()
w_hat = 0.0
u_mpc = 0.0
u_total = 0.0
t = 0.0

integral_e = 0.0
e_prev = x - np.zeros(2)  # 参考为零
t_last = -1.0  # 上次触发时间

# 记录
t_log = []
x_log = []
u_log = []
trigger_times = []

solve_count = 0
is_first = True

# ========== 主循环 ==========
while t <= t_final + 1e-9:
    # --- DOB 更新 ---
    # \dot{\hat{x}} = f(\hat{x}) + g(\hat{x})u + h(\hat{x})\hat{w} + L (x - \hat{x})
    f_hat = np.array([x_hat[1], -0.33 * np.exp(x_hat[0]) * x_hat[0] - 1.1 * x_hat[1]])
    g_hat = np.array([0.0, 1.0])
    h_hat = np.array([0.0, 1.0])
    dx_hat = f_hat + g_hat * u_total + h_hat * w_hat + L_gain @ (x - x_hat)
    # w_hat 更新: w_hat_dot = Gamma * h_hat^T * (x - x_hat)
    dw_hat = Gamma * h_hat.dot(x - x_hat)
    x_hat = x_hat + dt_sim * dx_hat
    w_hat = w_hat + dt_sim * dw_hat

    # 前馈补偿 (扰动匹配: u_dob = -w_hat)
    u_dob = -w_hat

    # --- 事件触发判断 ---
    e = x - np.zeros(2)
    integral_e = integral_e + e * dt_sim
    de = (e - e_prev) / dt_sim if dt_sim > 0 else np.zeros(2)
    J_PID = np.linalg.norm(Kp * e + Ki * integral_e + Kd * de)
    delta = delta0 - alpha * np.linalg.norm(x)
    if delta < 1e-3:
        delta = 1e-3  # 保证正值

    if is_first or (J_PID > delta):
        # 触发 MPC
        u_seq = solve_mpc(x)
        u_mpc = u_seq[0]
        integral_e = 0.0
        t_last = t
        trigger_times.append(t)
        solve_count += 1
        is_first = False

    # 总控制并限幅
    u_total = np.clip(u_dob + u_mpc, u_min, u_max)

    # --- 系统状态更新 (考虑扰动) ---
    w_dist = disturbance(t)
    x = rk4_step(x, u_total, w_dist, dt_sim)

    # 记录
    t_log.append(t)
    x_log.append(x.copy())
    u_log.append(u_total)

    e_prev = e
    t += dt_sim

# 转换为数组
t_log = np.array(t_log)
x_log = np.array(x_log)
u_log = np.array(u_log)

# ========== 绘图 ==========
plt.figure(figsize=(12, 4))
plt.subplot(1, 2, 1)
plt.plot(t_log, x_log[:, 0], "b-", linewidth=1.5, label="PID-DEMPC x1")
plt.plot(t_log, x_log[:, 1], "r--", linewidth=1.5, label="PID-DEMPC x2")
plt.xlabel("Time (s)")
plt.ylabel("States")
plt.legend()
plt.grid(True)
plt.title("State Trajectories")

plt.subplot(1, 2, 2)
plt.step(t_log, u_log, "k-", where="post", linewidth=1.2)
for tr in trigger_times:
    plt.axvline(x=tr, color="red", linestyle="--", alpha=0.5)
plt.xlabel("Time (s)")
plt.ylabel("Control Input u")
plt.title(f"Control Input & Trigger Instants (total MPC solves: {solve_count})")
plt.grid(True)
plt.tight_layout()
plt.show()

print(f"Total simulation time: {t_final}s")
print(f"Number of MPC solves: {solve_count}")
print(f"Trigger instants: {np.round(trigger_times, 3)}")
