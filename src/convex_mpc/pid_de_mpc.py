"""
PID-based Dual-mode Event-triggered MPC with Extended State Observer (ESO).

Implements the PID-DE-MPC algorithm for a 2D nonlinear system:

    x1_dot = x2
    x2_dot = -tao/M * exp(-x1) * x1 - hd/M * x2 + u/M + w(t)/M

The controller uses:
- LQR when near the origin (x'Px < r_trigger)
- Event-triggered MPC when far from origin
- PID-based trigger to reuse previous MPC solutions
- ESO for real-time disturbance estimation and compensation
"""

import casadi as ca
import numpy as np
from scipy.optimize import Bounds, minimize


class PIDDEMPC:
    """
    PID-DE-MPC controller for a 2D nonlinear system.

    Parameters
    ----------
    tao, M, hd : float
        System model parameters.
    Ts : float
        Sampling time [s].
    T : int
        MPC prediction horizon (number of steps).
    Q : (2,2) ndarray
        State cost matrix.
    R : float
        Input cost scalar.
    P : (2,2) ndarray
        Terminal cost matrix (also used in LQR and event-trigger).
    K : (2,) ndarray
        LQR feedback gain.
    w0 : float
        ESO bandwidth (beta1 = 2*w0, beta2 = w0^2).
    Kp, Ki, Kd : float
        PID event-trigger gains.
    r_trigger : float
        Terminal region threshold (x'Px < r_trigger activates LQR).
    pid_threshold : float
        PID trigger threshold.
    u_min, u_max : float
        Control input bounds.
    """

    def __init__(
        self,
        tao=0.9,
        M=1.25,
        hd=0.42,
        Ts=0.1,
        T=25,
        Q=None,
        R=None,
        P=None,
        K=None,
        w0=12,
        Kp=0.14,
        Ki=0.9,
        Kd=0.7,
        r_trigger=0.001508,
        pid_threshold=0.01,
        u_min=-1.0,
        u_max=1.0,
    ):
        self.tao = tao
        self.M = M
        self.hd = hd
        self.Ts = Ts
        self.T = T

        self.Q = Q if Q is not None else np.diag([0.2, 0.2])
        self.R_val = R if R is not None else 0.1
        self.P = P if P is not None else np.array([[0.1692, 0.0572], [0.0572, 0.1391]])
        self.K = K if K is not None else np.array([-0.4454, -1.0932])

        self.w0 = w0
        self.beta1 = 2.0 * w0
        self.beta2 = w0**2

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.r_trigger = r_trigger
        self.pid_threshold = pid_threshold

        self.u_min = u_min
        self.u_max = u_max

        # Internal state for event-triggered MPC
        self.xb = None  # Predicted state trajectory from last MPC solve (2, T)
        self.ucb = None  # Predicted control trajectory from last MPC solve (T,)
        self.tk = 1  # Number of controls already used from current solution
        self.JiF = 0.0  # Integral of trigger error
        self.event_times = []  # Time indices where MPC was re-solved

        # ESO state
        self.xhat = 0.0  # Estimated x2
        self.deso = 0.0  # Estimated disturbance
        self.udo = 0.0  # Disturbance compensation for control

        # Objective value log
        self.last_obj = None

        # Build MPC solver
        self._build_mpc_solver()

    def _f_dyn_np(self, x1, x2):
        """Numpy nonlinear dynamics drift term (numerically stable)."""
        # Clip x1 to prevent exp(-x1) overflow during optimisation
        x1_safe = np.clip(np.asarray(x1), -50.0, 50.0)
        return -self.tao / self.M * np.exp(-x1_safe) * x1_safe - self.hd / self.M * x2

    def _simulate_forward(self, x0, u_seq):
        """
        Simulate the system forward over the horizon given a control sequence.

        Returns
        -------
        x_traj : (2, T) ndarray
            State trajectory.
        obj : float
            Total cost (handles divergence gracefully).
        """
        T = self.T
        Ts = self.Ts
        x_traj = np.zeros((2, T))
        x_traj[:, 0] = np.clip(x0, -1e3, 1e3)

        obj = 0.0
        Q = self.Q
        R_val = self.R_val
        P = self.P
        state_limit = 1e4

        for k in range(T - 1):
            xk = x_traj[:, k]
            uk = np.clip(u_seq[k], self.u_min, self.u_max)

            obj += float(xk @ Q @ xk) + float(uk * R_val * uk)

            f_val = self._f_dyn_np(xk[0], xk[1])
            x_traj[0, k + 1] = xk[0] + Ts * xk[1]
            x_traj[1, k + 1] = xk[1] + Ts * (f_val + uk / self.M)

            # Clamp to prevent overflow
            x_traj[:, k + 1] = np.clip(x_traj[:, k + 1], -state_limit, state_limit)

            if not np.all(np.isfinite(x_traj[:, k + 1])):
                return x_traj, 1e12

        obj += float(x_traj[:, T - 1] @ P @ x_traj[:, T - 1])

        if not np.isfinite(obj) or obj > 1e10:
            obj = 1e12

        return x_traj, obj

    def _build_mpc_solver(self):
        """No CasADi build needed for scipy-based solver. Keep for compatibility."""
        self._last_u_guess = np.zeros(self.T)

    def solve_mpc(self, x_current):
        """
        Solve MPC using direct single shooting with scipy SLSQP.

        Parameters
        ----------
        x_current : (2,) ndarray
            Current state.

        Returns
        -------
        x_traj : (2, T) ndarray
            Predicted state trajectory.
        u_traj : (T,) ndarray
            Predicted control trajectory.
        obj_val : float
            Optimal cost.
        """
        x0_np = np.asarray(x_current).flatten()

        def cost(u_flat):
            _, obj = self._simulate_forward(x0_np, u_flat)
            return obj

        bounds = Bounds(np.full(self.T, self.u_min), np.full(self.T, self.u_max))

        # Warm-start from previous solution or LQR
        if self._last_u_guess is None or len(self._last_u_guess) != self.T:
            u_init = np.clip(self.K @ x0_np, self.u_min, self.u_max)
            u_init = np.full(self.T, u_init)
        else:
            # Shift previous solution
            u_init = np.roll(self._last_u_guess, -1)
            u_init[-1] = 0.0

        try:
            res = minimize(
                cost,
                u_init,
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": 200, "ftol": 1e-6, "disp": False},
            )
            u_opt = res.x
            obj_val = res.fun
            if not np.isfinite(obj_val):
                raise RuntimeError("Non-finite objective")
        except Exception:
            # Fall back to LQR
            u_lqr = np.clip(self.K @ x0_np, self.u_min, self.u_max)
            u_opt = np.full(self.T, u_lqr)
            obj_val = np.inf

        self._last_u_guess = u_opt.copy()

        # Simulate with optimal control to get state trajectory
        x_traj, _ = self._simulate_forward(x0_np, u_opt)
        self.last_obj = float(obj_val)

        return np.asarray(x_traj), np.asarray(u_opt), float(obj_val)

    def compute_control(self, x_current, i):
        """
        Compute control input for the current state.

        Parameters
        ----------
        x_current : (2,) ndarray
            Current state [x1, x2].
        i : int
            Current time step index (0-based, used for ESO initialisation).

        Returns
        -------
        u : float
            Control input to apply.
        info : dict
            Diagnostic information (mode, mpc_solved, events).
        """
        x_vec = np.asarray(x_current).flatten()
        P = self.P
        xPx = float(x_vec @ P @ x_vec)

        mode = "LQR"
        mpc_solved = False

        # ESO initialisation on first call
        if i == 0:
            self.xhat = float(x_vec[1])

        if xPx >= self.r_trigger:
            # ----- Event-triggered MPC region -----
            if self.xb is None:
                # First time entering MPC region: solve
                mode = "MPC_init"
                self.xb, self.ucb, obj = self.solve_mpc(x_vec)
                self.tk = 1
                self.JiF = 0.0
                self.event_times.append(i)
                mpc_solved = True
                u_mpc = float(self.ucb[0])
            else:
                # PID-based event trigger check
                e = x_vec - self.xb[:, self.tk]
                ePe = float(e @ P @ e)
                tk_plus_1 = self.tk + 1

                pid_val = (
                    np.exp(self.Kp) * ePe
                    + np.exp(self.Ki) * self.JiF
                    + np.exp(self.Kd) * ePe / tk_plus_1
                )

                if pid_val < self.pid_threshold and self.tk + 1 < self.T:
                    # Reuse next control from previous solution
                    mode = "MPC_reuse"
                    self.JiF += ePe * self.Ts
                    u_mpc = float(self.ucb[self.tk])
                    self.tk += 1
                else:
                    # Re-solve MPC
                    mode = "MPC_resolve"
                    self.event_times.append(i)
                    self.tk = 1
                    self.JiF = 0.0
                    self.xb, self.ucb, obj = self.solve_mpc(x_vec)
                    mpc_solved = True
                    u_mpc = float(self.ucb[0])
        else:
            # ----- Terminal region: LQR -----
            mode = "LQR"
            u_mpc = float(np.clip(self.K @ x_vec, self.u_min, self.u_max))
            self.xb = None

        # Disturbance compensation with output saturation
        u = float(np.clip(u_mpc - self.udo, self.u_min, self.u_max))

        return u, {"mode": mode, "mpc_solved": mpc_solved, "u_mpc": u_mpc}

    def update_eso(self, x_current, u_applied):
        """
        Update the Extended State Observer.

        Parameters
        ----------
        x_current : (2,) ndarray
            Current state [x1, x2] (state *before* control was applied).
        u_applied : float
            The control input that was actually applied.
        """
        x_vec = np.asarray(x_current).flatten()
        f_val = self._f_dyn_np(x_vec[0], x_vec[1])

        # ESO update (forward Euler)
        err = x_vec[1] - self.xhat
        self.xhat = self.xhat + self.Ts * (
            f_val + u_applied / self.M + self.deso / self.M + self.beta1 * err
        )
        self.deso = self.deso + self.Ts * (self.beta2 * err)

        # Clamp ESO states to prevent blowup
        self.xhat = float(np.clip(self.xhat, -1e6, 1e6))
        self.deso = float(np.clip(self.deso, -1e6, 1e6))
        self.udo = self.deso

    def reset(self, x0):
        """Reset internal state for a new simulation run."""
        self.xb = None
        self.ucb = None
        self.tk = 1
        self.JiF = 0.0
        self.event_times = []
        self.xhat = float(x0[1])
        self.deso = 0.0
        self.udo = 0.0
        self.last_obj = None


# ---------------------------------------------------------------------------
# Centroidal PID-DE-MPC: wraps CentroidalMPC with event-triggering and ESO
# ---------------------------------------------------------------------------


class CentroidalPIDDEMPC:
    """
    PID-DE-MPC wrapper for the Go2 centroidal MPC.

    Wraps CentroidalMPC with:
    - PID-based event-triggering to reduce QP solves
    - Extended State Observer (ESO) for 6-DOF disturbance estimation
    - Disturbance compensation fed into the MPC reference

    The event-trigger checks whether the current state stays close to the
    previously predicted trajectory. When it does, the previous solution is
    reused (shifted by one step). When it diverges, the MPC is re-solved.

    Compatible with ex00_demo.py patterns: exposes solve_QP with the same
    signature, plus solve_time / update_time attributes.
    """

    def __init__(
        self,
        go2,
        traj,
        mpc_dt=0.020833,  # default: GAIT_T/16 for GAIT_HZ=3
        w0=6.0,
        # --- Original parameters (higher solve rate) ---
        # Kp=0.05, Ki=0.3, Kd=0.15, pid_threshold=1.0
        #   exp(Kp)=1.051, exp(Ki)=1.350, exp(Kd)=1.162
        # --- Tuned for fewer solves than DEMPC/EMPC ---
        # Uses ||e|| (same scale as EMPC/DEMPC) instead of e'Qe.
        # Ki=-3 → exp(-3)=0.05, I term is 20x attenuated to prevent
        # unbounded accumulation from defeating the higher threshold.
        Kp=0.0,
        Ki=-3.0,
        Kd=0.0,
        pid_threshold=1.5,  # higher than EMPC's 0.8 → fewer triggers
        r_trigger=1e-6,
    ):
        from .centroidal_mpc import CentroidalMPC

        self.mpc = CentroidalMPC(go2, traj)
        self.go2 = go2
        self.N = traj.N

        # ESO parameters (6-DOF: linear + angular velocity)
        self.w0 = w0
        self.beta1 = 2.0 * w0
        self.beta2 = w0**2

        self.v_hat = np.zeros(6)
        self.d_hat = np.zeros(6)
        self.d_comp = np.zeros(6)
        self._alpha = 0.15  # low-pass filter gain
        self._Ts_eso = mpc_dt

        # PID event-trigger parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.pid_threshold = pid_threshold
        self.r_trigger = r_trigger

        # Trigger state
        self.xb = None
        self.ub = None
        self.tk = 1
        self.JiF = 0.0
        self.mpc_solve_count = 0
        self.mpc_reuse_count = 0
        self._solve_log = []  # list of (step_index, did_solve:bool)
        self._step_idx = 0

        # Timing
        self.solve_time = 0.0
        self.update_time = 0.0

    def solve_QP(self, go2, traj, verbose=False):
        """
        Solve or reuse MPC depending on PID event-trigger condition.

        Returns a dict with key 'x' mapping to a CasADi DM vector of
        stacked [X_opt, U_opt], matching CentroidalMPC.solve_QP format.

        The ESO disturbance estimate is applied as feedforward compensation
        on the first-step contact forces (distributed among stance legs).
        """
        import time

        t0 = time.perf_counter()
        self._step_idx += 1

        # ESO update: estimate disturbance from velocity tracking error
        # (run BEFORE solve so the estimate is available for compensation)
        x_current = go2.compute_com_x_vec().flatten()
        v_current = x_current[6:12]
        self._update_eso(v_current)

        Q_mat = self.mpc.Q
        xPx = float(x_current @ Q_mat @ x_current)

        did_solve = False

        if xPx < self.r_trigger or self.xb is None:
            # Terminal region or first call: always solve
            sol = self._do_solve(go2, traj, verbose)
            did_solve = True
            self._cache_solution(sol)
        else:
            # PID event-trigger check (uses ||e||, same scale as EMPC/DEMPC)
            e = x_current - self.xb[:, self.tk]
            e_norm = float(np.linalg.norm(e))
            tkp1 = float(self.tk + 1)

            pid_val = (
                np.exp(self.Kp) * e_norm
                + np.exp(self.Ki) * self.JiF
                + np.exp(self.Kd) * e_norm / tkp1
            )

            if pid_val < self.pid_threshold and self.tk + 1 < self.N:
                # Reuse: shift previous solution forward by one step
                self.JiF = min(self.JiF + e_norm * self._Ts_eso, 10.0)
                self.tk += 1
                self.mpc_reuse_count += 1
                sol = self._build_reuse_solution(self.tk)
            else:
                # Re-solve
                sol = self._do_solve(go2, traj, verbose)
                did_solve = True
                self._cache_solution(sol)

        t1 = time.perf_counter()

        # Apply disturbance compensation to first-step contact forces
        sol = self._apply_disturbance_compensation(sol, traj)

        # Propagate timing from underlying solver
        if did_solve:
            self.solve_time = self.mpc.solve_time
            self.mpc_solve_count += 1
        else:
            self.solve_time = 0.0
        self.update_time = (t1 - t0) * 1e3

        self._solve_log.append((self._step_idx, did_solve))

        return sol

    def _apply_disturbance_compensation(self, sol, traj):
        """
        Apply ESO disturbance estimate as feedforward force compensation.

        Distributes the estimated 6-DOF disturbance (force + torque) across
        stance legs in the first MPC step, matching the PID-DE-MPC principle:
        u_actual = u_mpc - d_hat.
        """
        # Only compensate if there is a meaningful estimate
        d_norm = np.linalg.norm(self.d_comp)
        if d_norm < 1e-6:
            return sol

        N = self.N
        w_opt = np.asarray(sol["x"]).flatten()
        U_opt = w_opt[12 * N :].reshape((12, N), order="F")

        # d_comp = [f_x, f_y, f_z, τ_x, τ_y, τ_z] on CoM
        d_force = self.d_comp[:3].copy()  # [f_x, f_y, f_z]

        # Distribute among stance legs at step 0
        ct = traj.contact_table
        stance_legs = [leg for leg in range(4) if ct[leg, 0] == 1]

        if len(stance_legs) == 0:
            return sol

        # Equal distribution per stance leg (z-compensation is primary)
        per_leg = d_force / len(stance_legs)

        for leg in stance_legs:
            base = 3 * leg
            U_opt[base : base + 3, 0] -= per_leg

        # Re-pack
        w_full = np.concatenate([w_opt[: 12 * N].flatten(), U_opt.flatten(order="F")])
        return {"x": ca.DM(w_full)}

    def _do_solve(self, go2, traj, verbose):
        """Perform an actual MPC solve via the underlying CentroidalMPC."""
        return self.mpc.solve_QP(go2, traj, verbose)

    def _cache_solution(self, sol):
        N = self.N
        w_opt = sol["x"].full().flatten()
        self.xb = w_opt[: 12 * N].reshape((12, N), order="F")
        self.ub = w_opt[12 * N :].reshape((12, N), order="F")
        self.tk = 1
        self.JiF = 0.0

    def _build_reuse_solution(self, offset):
        """Shift the cached trajectory forward so ``offset`` becomes index 0."""
        N = self.N
        x_reuse = np.zeros((12, N))
        u_reuse = np.zeros((12, N))

        for k in range(N):
            src = min(offset + k, N - 1)
            x_reuse[:, k] = self.xb[:, src]
            u_reuse[:, k] = self.ub[:, src]

        w_full = np.concatenate([x_reuse.flatten(order="F"), u_reuse.flatten(order="F")])
        return {"x": ca.DM(w_full)}

    def _update_eso(self, v_current):
        """ESO update for 6-DOF velocity disturbance estimation."""
        Ts = self._Ts_eso
        v_hat_prev = self.v_hat.copy()
        self.v_hat += Ts * (self.beta1 * (v_current - self.v_hat) + self.d_hat)
        self.d_hat += Ts * (self.beta2 * (v_current - v_hat_prev))
        self.d_comp = (1.0 - self._alpha) * self.d_comp + self._alpha * self.d_hat
