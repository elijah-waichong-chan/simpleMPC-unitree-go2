#include "locomotion_controller/gait.hpp"

#include <algorithm>
#include <cmath>

namespace locomotion_mpc {

namespace {
const Eigen::Vector4d kPhaseOffset(0.5, 0.0, 0.0, 0.5);
constexpr double kHeightSwing = 0.1;
}  // namespace

Gait::Gait(double frequency_hz, double duty)
  : gait_duty_(duty), gait_hz_(frequency_hz)
{
  gait_period_ = 1.0 / gait_hz_;
  stance_time_ = gait_duty_ * gait_period_;
  swing_time_ = (1.0 - gait_duty_) * gait_period_;
}

Eigen::Matrix<int, 4, Eigen::Dynamic>
Gait::computeContactTable(double t0, double dt, int N) const
{
  Eigen::Matrix<int, 4, Eigen::Dynamic> contact(4, N);
  for (int k = 0; k < N; ++k) {
    double t = t0 + static_cast<double>(k) * dt + dt * 0.5;
    for (int i = 0; i < 4; ++i) {
      double phase = std::fmod(kPhaseOffset[i] + t / gait_period_, 1.0);
      if (phase < 0.0) phase += 1.0;
      contact(i, k) = (phase < gait_duty_) ? 1 : 0;
    }
  }
  return contact;
}

Eigen::Vector4i Gait::computeCurrentMask(double time) const
{
  auto table = computeContactTable(time, 0.0, 1);
  return table.col(0);
}

Eigen::Vector3d Gait::computeTouchdownWorldForTrajPurposeOnly(
  const PinocchioModel & go2,
  const std::string & leg) const
{
  const Eigen::Vector3d & base_pos = go2.currentConfig().basePos();
  const Eigen::Vector3d & base_vel = go2.currentConfig().baseVel();
  const Eigen::Matrix3d & R_z = go2.Rz();
  const double yaw_rate = go2.yawRateDesWorld();

  const Eigen::Vector3d hip_offset = go2.getHipOffset(leg);
  const Eigen::Vector3d body_pos(base_pos.x(), base_pos.y(), 0.0);
  const Eigen::Vector3d hip_pos_world = body_pos + R_z * hip_offset;

  const double T = swing_time_ + 0.5 * stance_time_;
  const double pred_time = 0.5 * T;

  const Eigen::Vector3d pos_nominal(hip_pos_world.x(), hip_pos_world.y(), 0.02);
  const Eigen::Vector3d pos_drift(base_vel.x() * pred_time, base_vel.y() * pred_time, 0.0);

  const double dtheta = yaw_rate * pred_time;
  const Eigen::Vector2d center_xy(base_pos.x(), base_pos.y());
  const Eigen::Vector2d r_xy(pos_nominal.x(), pos_nominal.y());
  const Eigen::Vector2d r_rel = r_xy - center_xy;
  const Eigen::Vector3d rot_corr(-dtheta * r_rel.y(), dtheta * r_rel.x(), 0.0);

  return pos_nominal + pos_drift + rot_corr;
}

std::pair<SwingTrajectory, Eigen::Vector3d>
Gait::computeSwingTrajAndTouchdown(const PinocchioModel & go2,
                                   const std::string & leg) const
{
  const Eigen::Vector3d & base_pos = go2.currentConfig().basePos();
  const Eigen::Vector3d & pos_com_world = go2.posComWorld();
  const Eigen::Vector3d & vel_com_world = go2.velComWorld();
  const Eigen::Matrix3d & R_z = go2.Rz();
  const double yaw_rate = go2.yawRateDesWorld();

  const Eigen::Vector3d hip_offset = go2.getHipOffset(leg);
  const auto foot_state = go2.getSingleFootStateInWorld(leg);
  const Eigen::Vector3d foot_pos = foot_state.first;

  const Eigen::Vector3d body_pos(base_pos.x(), base_pos.y(), 0.0);
  const Eigen::Vector3d hip_pos_world = body_pos + R_z * hip_offset;

  const double x_vel_des = go2.xVelDesWorld();
  const double y_vel_des = go2.yVelDesWorld();
  const double x_pos_des = go2.xPosDesWorld();
  const double y_pos_des = go2.yPosDesWorld();

  const double T = swing_time_ + 0.5 * stance_time_;
  const double pred_time = 0.5 * T;

  const double k_v_x = 0.4 * T;
  const double k_p_x = 0.1;
  const double k_v_y = 0.2 * T;
  const double k_p_y = 0.05;

  const Eigen::Vector3d pos_nominal(hip_pos_world.x(), hip_pos_world.y(), 0.02);
  const Eigen::Vector3d pos_drift(x_vel_des * pred_time, y_vel_des * pred_time, 0.0);
  const Eigen::Vector3d pos_correction(
    k_p_x * (pos_com_world.x() - x_pos_des),
    k_p_y * (pos_com_world.y() - y_pos_des),
    0.0);
  const Eigen::Vector3d vel_correction(
    k_v_x * (vel_com_world.x() - x_vel_des),
    k_v_y * (vel_com_world.y() - y_vel_des),
    0.0);

  const double dtheta = yaw_rate * pred_time;
  const Eigen::Vector2d center_xy(base_pos.x(), base_pos.y());
  const Eigen::Vector2d r_xy(pos_nominal.x(), pos_nominal.y());
  const Eigen::Vector2d r_rel = r_xy - center_xy;
  const Eigen::Vector3d rot_corr(-dtheta * r_rel.y(), dtheta * r_rel.x(), 0.0);

  const Eigen::Vector3d pos_touchdown =
    pos_nominal + pos_drift + pos_correction + vel_correction + rot_corr;

  SwingTrajectory traj(foot_pos, pos_touchdown, swing_time_, kHeightSwing);
  return {traj, pos_touchdown};
}

}  // namespace locomotion_mpc
