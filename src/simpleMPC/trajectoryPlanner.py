import numpy as np
from robot_classes import RigidBodyState
from robot_classes import RigidBodyTraj

from readURDF import createFloatingBaseModel
from readURDF import getFootPlacement
from readURDF import getHipOffset
import pinocchio as pin 

def walkGait(frequency_hz: float,
             duty: float,
             time_step: float,
             time_now: float,
             time_horizon: float) -> np.ndarray:
    
    N = int(np.ceil(time_horizon / time_step)) # number of sequences to output
    t = time_now + np.arange(N) * time_step # time vector
    T = 1 / frequency_hz # Perioid

    phase_offset = np.array([0.00, 0.25, 0.50, 0.75])  # FL, FR, RL, RR
    
    phases = np.mod(t[None, :] / T + phase_offset[:, None], 1.0)
    C = (phases < duty).astype(np.int32)  # 4 x N

    return C

def raibertFootPlacement(p_com, x_vel, y_vel, frequency, duty):
    period = 1/frequency
    stanceTime = duty * period
    swingTime = (1-duty) * period

    r_next_touchdown = p_com + np.array([x_vel*(swingTime + stanceTime/2), y_vel*(swingTime + stanceTime/2), 0])
    return r_next_touchdown

def generateConstantTraj(state: RigidBodyState,
                 x_vel: float,
                 y_vel: float,
                 z_position: float,
                 yaw_rate: float,
                 time_step: float,
                 time_horizon: float,
                 frequency: float,
                 duty: float) -> RigidBodyTraj:
    
    N = int(time_horizon / time_step) # number of sequences to output
    t_vec = np.arange(N) * time_step # time vector

    trajectory = RigidBodyTraj()

    trajectory.time = t_vec

    trajectory.x_pos_ref = state.x_pos + x_vel * t_vec
    trajectory.x_vel_ref = np.full(N, x_vel, dtype=float)

    trajectory.y_pos_ref = state.y_pos + y_vel * t_vec
    trajectory.y_vel_ref = np.full(N, y_vel, dtype=float)

    trajectory.z_pos_ref = np.full(N, z_position, dtype=float)
    trajectory.z_vel_ref = np.full(N, 0, dtype=float)

    trajectory.yaw_ref = state.yaw + yaw_rate * t_vec
    trajectory.yaw_rate_ref = np.full(N, yaw_rate, dtype=float)

    trajectory.pitch_ref = state.pitch
    trajectory.pitch_rate_ref = np.full(N, 0, dtype=float)

    trajectory.roll_ref = state.roll
    trajectory.roll_rate_ref = np.full(N, 0, dtype=float)

    model, data, vmodel, vdata, cmodel, cdata  = createFloatingBaseModel()
    q = pin.neutral(model)
    [r_fl_next, r_fr_next, r_rl_next, r_rr_next] = getFootPlacement(model, data, q)

    pin.centerOfMass(model, data)
    com_world = data.com[0]

    schedule= walkGait(frequency, duty, time_step, 0, time_horizon)
    print(schedule)

    r_fl = np.zeros((3,N))
    r_fr = np.zeros((3,N))
    r_rl = np.zeros((3,N))
    r_rr = np.zeros((3,N))

    r_fl_vec = np.zeros((3,N))
    r_fr_vec = np.zeros((3,N))
    r_rl_vec = np.zeros((3,N))
    r_rr_vec = np.zeros((3,N))
    [fl_hip_offset, fr_hip_offset, rl_hip_offset, rr_hip_offset] = getHipOffset(model, data, q)


    mask_previous = np.array([2,2,2,2])


    for i in range(N):
        current_mask = schedule[:, i]
        p_com = np.array([com_world[0] + x_vel*i*time_step, com_world[1] + y_vel*i*time_step, com_world[2]])


        if current_mask[0] != mask_previous[0]:
            if current_mask[0] == 0:
                r_fl_next = fl_hip_offset + raibertFootPlacement(np.array([p_com[0], p_com[1],r_fl[2, i-1]]), x_vel, y_vel, frequency, duty)
                r_fl[:,i] = np.array([0,0,0])
                r_fl_vec[:,i] = np.array([0,0,0])
            elif current_mask[0] == 1:
                r_fl[:,i] = r_fl_next
        else:
            r_fl[:,i] = r_fl[:,i-1] 
        if current_mask[0] == 1:
            r_fl_vec[:,i] = r_fl[:,i] - p_com

        if current_mask[1] != mask_previous[1]:
            if current_mask[1] == 0:
                r_fr_next = fr_hip_offset + raibertFootPlacement(np.array([p_com[0], p_com[1],r_fl[2, i-1]]), x_vel, y_vel, frequency, duty)
                r_fr[:,i] = np.array([0,0,0])
                r_fr_vec[:,i] = np.array([0,0,0])
            elif current_mask[1] == 1:
                r_fr[:,i] = r_fr_next
        else:
            r_fr[:,i] = r_fr[:,i-1] 
        if current_mask[1] == 1:
            r_fr_vec[:,i] = r_fr[:,i] - p_com

        if current_mask[2] != mask_previous[2]:
            if current_mask[2] == 0:
                r_rl_next = rl_hip_offset + raibertFootPlacement(np.array([p_com[0], p_com[1],r_fl[2, i-1]]), x_vel, y_vel, frequency, duty)
                r_rl[:,i] = np.array([0,0,0])
                r_rl_vec[:,i] = np.array([0,0,0])
            elif current_mask[2] == 1:
                r_rl[:,i] = r_rl_next
        else:
            r_rl[:,i] = r_rl[:,i-1] 
        if current_mask[2] == 1:
            r_rl_vec[:,i] = r_rl[:,i] - p_com

        if current_mask[3] != mask_previous[3]:
            if current_mask[3] == 0:
                r_rr_next = rr_hip_offset + raibertFootPlacement(np.array([p_com[0], p_com[1],r_fl[2, i-1]]), x_vel, y_vel, frequency, duty)
                r_rr[:,i] = np.array([0,0,0])
                r_rr_vec[:,i] = np.array([0,0,0])
            elif current_mask[3] == 1:
                r_rr[:,i] = r_rr_next
        else:
            r_rr[:,i] = r_rr[:,i-1] 
        if current_mask[3] == 1:
            r_rr_vec[:,i] = r_rr[:,i] - p_com

        mask_previous = current_mask

    trajectory.fl_foot_placement = r_fl
    trajectory.fr_foot_placement = r_fr
    trajectory.rl_foot_placement = r_rl
    trajectory.rr_foot_placement = r_rr

    trajectory.fl_foot_placement_vec = r_fl_vec
    trajectory.fr_foot_placement_vec = r_fr_vec
    trajectory.rl_foot_placement_vec = r_rl_vec
    trajectory.rr_foot_placement_vec = r_rr_vec

    return trajectory