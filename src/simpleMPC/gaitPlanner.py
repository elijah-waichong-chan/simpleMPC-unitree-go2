import numpy as np

def walkGait(frequency_hz: float,
             duty: float,
             time_step: int,
             time_now: float,
             time_horizon: float) -> np.ndarray:
    
    N = int(np.ceil(time_horizon / time_step)) # number of sequences to output
    t = time_now + np.arange(N) * time_step # time vector
    T = 1 / frequency_hz # Perioid

    phase_offset = np.array([0.00, 0.25, 0.50, 0.75])  # FL, FR, RL, RR
    
    phases = np.mod(t[None, :] / T + phase_offset[:, None], 1.0)
    C = (phases < duty).astype(np.int32)  # 4 x N

    return C

# Test Code
schedule = walkGait(0.6, 0.8, 0.1, 0, 3)
print(schedule)
