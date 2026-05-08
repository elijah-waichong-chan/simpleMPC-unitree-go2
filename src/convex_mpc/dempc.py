"""
Disturbance-Observer-Based Event-Triggered MPC (DEMPC).

Wraps CentroidalMPC with:
  - Static event-triggering (same threshold as EMPC)
  - Extended State Observer (ESO) for 6-DOF disturbance estimation
  - Disturbance feedforward compensation on contact forces

Paper reference (Section IV):
  DEMPC = EMPC + DOB (static threshold, with disturbance observer).

Compared to PID-DE-MPC:
  - Same ESO + disturbance compensation
  - Static threshold instead of PID-based dynamic threshold
"""

import casadi as ca
import numpy as np


class CentroidalDEMPC:
    """
    DEMPC wrapper for the Go2 centroidal MPC.

    Static event-triggered MPC with disturbance observer:
    re-solves the QP when ||x - x_pred|| > static_threshold, and
    compensates contact forces with the ESO disturbance estimate.
    """

    def __init__(
        self,
        go2,
        traj,
        mpc_dt=0.020833,
        w0=6.0,
        static_threshold=0.8,
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

        # Event-trigger parameters (static)
        self.static_threshold = static_threshold

        # Trigger state
        self.xb = None
        self.ub = None
        self.tk = 1
        self.mpc_solve_count = 0
        self.mpc_reuse_count = 0
        self._step_idx = 0

        # Timing
        self.solve_time = 0.0
        self.update_time = 0.0

    def solve_QP(self, go2, traj, verbose=False):
        """
        Solve or reuse MPC depending on static event-trigger condition.

        The ESO disturbance estimate is applied as feedforward compensation
        on the first-step contact forces.
        """
        import time

        t0 = time.perf_counter()
        self._step_idx += 1

        # ESO update (before solve, so estimate is available for compensation)
        x_current = go2.compute_com_x_vec().flatten()
        v_current = x_current[6:12]
        self._update_eso(v_current)

        did_solve = False

        if self.xb is None:
            sol = self._do_solve(go2, traj, verbose)
            did_solve = True
            self._cache_solution(sol)
        else:
            e = x_current - self.xb[:, self.tk]
            e_norm = float(np.linalg.norm(e))

            if e_norm <= self.static_threshold and self.tk + 1 < self.N:
                self.mpc_reuse_count += 1
                self.tk += 1
                sol = self._build_reuse_solution(self.tk)
            else:
                sol = self._do_solve(go2, traj, verbose)
                did_solve = True
                self._cache_solution(sol)

        t1 = time.perf_counter()

        # Apply disturbance compensation
        sol = self._apply_disturbance_compensation(sol, traj)

        if did_solve:
            self.solve_time = self.mpc.solve_time
            self.mpc_solve_count += 1
        else:
            self.solve_time = 0.0
        self.update_time = (t1 - t0) * 1e3

        return sol

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _do_solve(self, go2, traj, verbose):
        return self.mpc.solve_QP(go2, traj, verbose)

    def _cache_solution(self, sol):
        N = self.N
        w_opt = sol["x"].full().flatten()
        self.xb = w_opt[: 12 * N].reshape((12, N), order="F")
        self.ub = w_opt[12 * N :].reshape((12, N), order="F")
        self.tk = 1

    def _build_reuse_solution(self, offset):
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

    def _apply_disturbance_compensation(self, sol, traj):
        """Distribute estimated CoM force disturbance among stance legs."""
        d_norm = np.linalg.norm(self.d_comp)
        if d_norm < 1e-6:
            return sol

        N = self.N
        w_opt = np.asarray(sol["x"]).flatten()
        U_opt = w_opt[12 * N :].reshape((12, N), order="F")

        d_force = self.d_comp[:3].copy()

        ct = traj.contact_table
        stance_legs = [leg for leg in range(4) if ct[leg, 0] == 1]
        if len(stance_legs) == 0:
            return sol

        per_leg = d_force / len(stance_legs)
        for leg in stance_legs:
            base = 3 * leg
            U_opt[base : base + 3, 0] -= per_leg

        w_full = np.concatenate([w_opt[: 12 * N].flatten(), U_opt.flatten(order="F")])
        return {"x": ca.DM(w_full)}
