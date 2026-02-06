float64 sim_time

# Generalized coordinates (q), length = 19
# Order:
# 0–2   : base position (x, y, z)
# 3–6   : base orientation (quaternion: qw, qx, qy, qz)
# 7–9   : FL hip, knee, ankle
# 10–12 : FR hip, knee, ankle
# 13–15 : RL hip, knee, ankle
# 16–18 : RR hip, knee, ankle
float64[19] q

# Generalized velocities (dq), length = 18
# Order:
# 0–2   : base linear velocity (vx, vy, vz)
# 3–5   : base angular velocity (wx, wy, wz)
# 6–8   : FL hip, knee, ankle
# 9–11  : FR hip, knee, ankle
# 12–14 : RL hip, knee, ankle
# 15–17 : RR hip, knee, ankle
float64[18] dq
