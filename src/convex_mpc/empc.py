"""
Event-Triggered MPC (EMPC) with static threshold.

Wraps CentroidalMPC with a static event-triggering mechanism that only
re-solves the QP when the state prediction error exceeds a fixed threshold.

Paper reference (Section IV):
  Triggering condition: ||e(t)|| > 0.8  (static)
  No disturbance observer.

Compared to PID-DE-MPC:
  - Same event-triggered reuse mechanism (shift cached solution forward)
  - Static threshold instead of PID-based dynamic threshold
  - No ESO / disturbance compensation
"""

import casadi as ca
import numpy as np


class CentroidalEMPC:
    """
    Event-Triggered MPC wrapper for the Go2 centroidal MPC.

    Uses a static threshold: re-solves the QP only when the weighted
    state prediction error exceeds ``static_threshold``.  Otherwise the
    previous solution is shifted forward and reused.
    """

    def __init__(
        self,
        go2,
        traj,
        mpc_dt=0.020833,
        static_threshold=0.8,
    ):
        from .centroidal_mpc import CentroidalMPC

        self.mpc = CentroidalMPC(go2, traj)
        self.go2 = go2
        self.N = traj.N

        # Event-trigger parameters
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

        Returns a dict with key 'x' mapping to a CasADi DM vector of
        stacked [X_opt, U_opt], matching CentroidalMPC.solve_QP format.
        """
        import time

        t0 = time.perf_counter()
        self._step_idx += 1

        x_current = go2.compute_com_x_vec().flatten()
        did_solve = False

        if self.xb is None:
            # First call: always solve
            sol = self._do_solve(go2, traj, verbose)
            did_solve = True
            self._cache_solution(sol)
        else:
            # Static event-trigger check
            e = x_current - self.xb[:, self.tk]
            e_norm = float(np.linalg.norm(e))

            if e_norm <= self.static_threshold and self.tk + 1 < self.N:
                # Reuse: shift previous solution forward
                self.mpc_reuse_count += 1
                self.tk += 1
                sol = self._build_reuse_solution(self.tk)
            else:
                # Re-solve
                sol = self._do_solve(go2, traj, verbose)
                did_solve = True
                self._cache_solution(sol)

        t1 = time.perf_counter()

        if did_solve:
            self.solve_time = self.mpc.solve_time
            self.mpc_solve_count += 1
        else:
            self.solve_time = 0.0
        self.update_time = (t1 - t0) * 1e3

        return sol

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
