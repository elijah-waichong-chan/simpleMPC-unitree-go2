float64 sim_time

# Generalized coordinates (q), length = 19
# Order:
# 0–2   : base position in world frame (x, y, z)
# 3–6   : base orientation (quaternion: qw, qx, qy, qz)
# 7–9   : FL hip, thigh, calf
# 10–12 : FR hip, thigh, calf
# 13–15 : RL hip, thigh, calf
# 16–18 : RR hip, thigh, calf
float64[19] q

# Generalized velocities (dq), length = 18
# Order:
# 0–2   : base linear velocity in world frame (vx, vy, vz)
# 3–5   : base angular velocity in body frame (wx, wy, wz)
# 6–8   : FL hip, thigh, calf
# 9–11  : FR hip, thigh, calf
# 12–14 : RL hip, thigh, calf
# 15–17 : RR hip, thigh, calf
float64[18] dq
