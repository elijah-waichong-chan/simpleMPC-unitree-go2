import numpy as np
import time
from collections import defaultdict
from .go2_robot_data import PinGo2Model
from .gait import Gait
from dataclasses import dataclass


# --------------------------------------------------------------------------------
# Leg Controller Setting
# --------------------------------------------------------------------------------

# KP_SWING = np.diag([400, 400, 400])
# KD_SWING = np.diag([75, 75, 75])

KP_SWING = np.diag([200, 200, 200])
KD_SWING = np.diag([35, 35, 35])

# Mapping from leg name to index in the mask
LEG_INDEX = {
    "FL": 0,
    "FR": 1,
    "RL": 2,
    "RR": 3,
}

# Mapping from leg name to the joint torque slice in (C*dq + g)
JOINT_SLICES = {
    "FL": slice(6, 9),
    "FR": slice(9, 12),
    "RL": slice(12, 15),
    "RR": slice(15, 18),
}

LEG_FORCE_SLICES = {
    "FL": slice(0, 3),
    "FR": slice(3, 6),
    "RL": slice(6, 9),
    "RR": slice(9, 12),
}

@dataclass
class LegOutput:
    tau: np.ndarray       # shape (3,)
    pos_des: np.ndarray   # shape (3,)
    pos_now: np.ndarray   # shape (3,)
    vel_des: np.ndarray   # shape (3,)
    vel_now: np.ndarray   # shape (3,)


class LegController():
        
    def __init__(self):
            self.last_mask = np.array([2, 2, 2, 2])
            self._timing_enabled = False
            self._timing_period_s = 1.0
            self._timing_last_log = time.perf_counter()
            self._timing_acc = {leg: defaultdict(float) for leg in LEG_INDEX.keys()}
            self._timing_count = {leg: 0 for leg in LEG_INDEX.keys()}
            self.current_mask = None

    def compute_leg_torque(
        self,
        leg: str,
        go2: PinGo2Model,
        gait: Gait,
        contact_force: np.ndarray,
        current_time: float,
    ):
        t0 = time.perf_counter() if self._timing_enabled else None
        # Extract Parameters
        leg_idx = LEG_INDEX[leg]
        if self.current_mask is not None:
            current_mask = self.current_mask
        else:
            current_mask = gait.compute_current_mask(current_time).reshape(-1)
        tau_cmd = np.zeros((3, 1))

        # Coompute Foot Jacobian
        t_seg = time.perf_counter() if self._timing_enabled else None
        J_foot_world = go2.compute_3x3_foot_Jacobian_world(leg)      # (3x3)
        if self._timing_enabled:
            jac_dyn_s = time.perf_counter() - t_seg

        # Initialize desired to current
        t_seg = time.perf_counter() if self._timing_enabled else None
        foot_pos_now, foot_vel_now = go2.get_single_foot_state_in_world(leg)
        foot_pos_des, foot_vel_des = foot_pos_now, foot_vel_now
        if self._timing_enabled:
            foot_init_s = time.perf_counter() - t_seg

        t_seg = time.perf_counter() if self._timing_enabled else None
        # Detect takeoff transition
        if self.last_mask[leg_idx] != current_mask[leg_idx] and current_mask[leg_idx] == 0:
            # This leg just took off
            setattr(self, f"{leg}_takeoff_time", current_time)
            traj, td_pos = gait.compute_swing_traj_and_touchdown(go2, leg)
            setattr(self, f"{leg}_traj", traj)
            setattr(self, f"{leg}_td_pos", td_pos)
        if self._timing_enabled:
            takeoff_s = time.perf_counter() - t_seg

        # Swing vs stance
        if current_mask[leg_idx] == 0:  # Swing phase
            t_seg = time.perf_counter() if self._timing_enabled else None
            takeoff_time = getattr(self, f"{leg}_takeoff_time")
            traj = getattr(self, f"{leg}_traj")

            time_since_takeoff = current_time - takeoff_time
            foot_pos_des, foot_vel_des, foot_acl_des = traj(time_since_takeoff)

            pos_error = foot_pos_des - foot_pos_now
            vel_error = foot_vel_des - foot_vel_now
            if self._timing_enabled:
                swing_traj_s = time.perf_counter() - t_seg

            # PD in Cartesian space
            t_seg = time.perf_counter() if self._timing_enabled else None
            force = KP_SWING @ pos_error + KD_SWING @ vel_error  # (3x1)
            tau_cmd = J_foot_world.T @ force
            if self._timing_enabled:
                map_tau_s = time.perf_counter() - t_seg

        else:  # Stance phase
            t_seg = time.perf_counter() if self._timing_enabled else None
            tau_cmd = J_foot_world.T @ -contact_force
            if self._timing_enabled:
                stance_tau_s = time.perf_counter() - t_seg

        # Update mask memory
        self.last_mask[leg_idx] = current_mask[leg_idx]

        if self._timing_enabled:
            total_s = time.perf_counter() - t0
            acc = self._timing_acc[leg]
            acc["jac_dyn"] = max(acc["jac_dyn"], jac_dyn_s)
            acc["foot_init"] = max(acc["foot_init"], foot_init_s)
            acc["takeoff"] = max(acc["takeoff"], takeoff_s)
            if current_mask[leg_idx] == 0:
                acc["swing_traj"] = max(acc["swing_traj"], swing_traj_s)
                acc["map_tau"] = max(acc["map_tau"], map_tau_s)
            else:
                acc["stance_tau"] = max(acc["stance_tau"], stance_tau_s)
            acc["total"] = max(acc["total"], total_s)
            self._timing_count[leg] += 1

            now = time.perf_counter()
            if now - self._timing_last_log >= self._timing_period_s:
                for leg_name in LEG_INDEX.keys():
                    n = self._timing_count[leg_name]
                    if n == 0:
                        continue
                    a = self._timing_acc[leg_name]
                    def max_ms(key):
                        return a.get(key, 0.0) * 1000.0
                    print(
                        f"[leg_timing] {leg_name} "
                        f"total={max_ms('total'):.3f} "
                        f"jac_dyn={max_ms('jac_dyn'):.3f} "
                        f"foot_init={max_ms('foot_init'):.3f} "
                        f"takeoff={max_ms('takeoff'):.3f} "
                        f"swing_traj={max_ms('swing_traj'):.3f} "
                        f"map_tau={max_ms('map_tau'):.3f} "
                        f"stance_tau={max_ms('stance_tau'):.3f}"
                    )
                self._timing_last_log = now
                self._timing_acc = {leg: defaultdict(float) for leg in LEG_INDEX.keys()}
                self._timing_count = {leg: 0 for leg in LEG_INDEX.keys()}

        return LegOutput(
            tau=tau_cmd.reshape(3,),
            pos_des=foot_pos_des,
            pos_now=foot_pos_now,
            vel_des=foot_vel_des,
            vel_now=foot_vel_now,
        )

    def compute_all_leg_torques(self, go2, gait, contact_forces_world, current_time):
        forces = np.asarray(contact_forces_world, dtype=float).reshape(-1)
        self.current_mask = gait.compute_current_mask(current_time).reshape(-1)

        tau_raw = np.zeros(12, dtype=float)
        for leg in ("FL", "FR", "RL", "RR"):
            f_leg = forces[LEG_FORCE_SLICES[leg]]
            leg_out = self.compute_leg_torque(leg, go2, gait, f_leg, current_time)
            tau_raw[LEG_FORCE_SLICES[leg]] = leg_out.tau
        return tau_raw
