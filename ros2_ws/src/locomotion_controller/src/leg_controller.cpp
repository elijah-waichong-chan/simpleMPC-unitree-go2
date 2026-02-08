#include "locomotion_controller/leg_controller.hpp"

#include <stdexcept>

namespace locomotion_mpc {

LegController::LegController()
{
  takeoff_time_.fill(0.0);
  td_pos_.fill(Eigen::Vector3d::Zero());
}

void LegController::setSwingGains(double kp, double kd)
{
  swing_kp_ = kp;
  swing_kd_ = kd;
}

LegController::LegIndex LegController::legIndexFromName(const std::string & leg)
{
  if (leg == "FL") return FL;
  if (leg == "FR") return FR;
  if (leg == "RL") return RL;
  if (leg == "RR") return RR;
  throw std::invalid_argument("Unknown leg name: " + leg);
}

LegOutput LegController::computeLegTorque(const std::string & leg,
                                          PinocchioModel & go2,
                                          const Gait & gait,
                                          const Eigen::Vector3d & contact_force,
                                          double current_time)
{
  LegIndex idx = legIndexFromName(leg);
  Eigen::Vector4i current_mask = has_current_mask_ ? current_mask_ : gait.computeCurrentMask(current_time);

  Eigen::Matrix3d J_foot_world = go2.compute3x3FootJacobianWorld(leg);

  auto foot_state = go2.getSingleFootStateInWorld(leg);
  Eigen::Vector3d foot_pos_now = foot_state.first;
  Eigen::Vector3d foot_vel_now = foot_state.second;
  Eigen::Vector3d foot_pos_des = foot_pos_now;
  Eigen::Vector3d foot_vel_des = foot_vel_now;

  if (last_mask_(idx) != current_mask(idx) && current_mask(idx) == 0) {
    takeoff_time_[idx] = current_time;
    auto swing = gait.computeSwingTrajAndTouchdown(go2, leg);
    swing_traj_[idx] = swing.first;
    td_pos_[idx] = swing.second;
  }

  Eigen::Vector3d tau_cmd = Eigen::Vector3d::Zero();

  if (current_mask(idx) == 0) {
    double t = current_time - takeoff_time_[idx];
    Eigen::Vector3d foot_acc_des;
    swing_traj_[idx].eval(t, foot_pos_des, foot_vel_des, foot_acc_des);

    Eigen::Vector3d pos_error = foot_pos_des - foot_pos_now;
    Eigen::Vector3d vel_error = foot_vel_des - foot_vel_now;

    const Eigen::Matrix<double, 3, Eigen::Dynamic> J_full =
      go2.computeFullFootJacobianWorld(leg);
    const Eigen::Vector3d Jdot_dq = go2.computeJdotDqWorld(leg);

    Eigen::VectorXd g;
    Eigen::MatrixXd C;
    Eigen::MatrixXd M;
    go2.computeDynamicsTerms(g, C, M);

    const Eigen::VectorXd dq = go2.currentConfig().getDq();
    const Eigen::VectorXd cdq_g = C * dq + g;
    const int joint_start = 6 + static_cast<int>(idx) * 3;
    const Eigen::Vector3d cdq_g_leg = cdq_g.segment<3>(joint_start);

    const Eigen::MatrixXd Minv_Jt = M.ldlt().solve(J_full.transpose());
    const Eigen::Matrix3d Lambda = (J_full * Minv_Jt).inverse();
    const Eigen::Vector3d f_ff = Lambda * (foot_acc_des - Jdot_dq);

    Eigen::Vector3d force = swing_kp_ * pos_error + swing_kd_ * vel_error + f_ff;
    tau_cmd = J_foot_world.transpose() * force + cdq_g_leg;
  } else {
    tau_cmd = J_foot_world.transpose() * (-contact_force);
  }

  last_mask_(idx) = current_mask(idx);

  LegOutput output;
  output.tau = tau_cmd;
  output.pos_des = foot_pos_des;
  output.pos_now = foot_pos_now;
  output.vel_des = foot_vel_des;
  output.vel_now = foot_vel_now;
  return output;
}

Eigen::VectorXd LegController::computeAllLegTorques(PinocchioModel & go2,
                                                    const Gait & gait,
                                                    const Eigen::VectorXd & contact_forces_world,
                                                    double current_time)
{
  Eigen::VectorXd forces = contact_forces_world;
  forces.resize(12);
  current_mask_ = gait.computeCurrentMask(current_time);
  has_current_mask_ = true;

  Eigen::VectorXd tau_raw(12);
  tau_raw.setZero();

  const std::array<std::string, 4> legs = {"FL", "FR", "RL", "RR"};
  for (size_t i = 0; i < legs.size(); ++i) {
    Eigen::Vector3d f_leg = forces.segment<3>(static_cast<int>(i) * 3);
    LegOutput leg_out = computeLegTorque(legs[i], go2, gait, f_leg, current_time);
    tau_raw.segment<3>(static_cast<int>(i) * 3) = leg_out.tau;
  }

  has_current_mask_ = false;
  return tau_raw;
}

}  // namespace locomotion_mpc
