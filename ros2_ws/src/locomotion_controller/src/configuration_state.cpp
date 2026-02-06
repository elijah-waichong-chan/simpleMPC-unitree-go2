#include "locomotion_controller/configuration_state.hpp"

#include <cmath>
#include <stdexcept>

#include <pinocchio/math/rpy.hpp>

namespace locomotion_mpc {

ConfigurationState::ConfigurationState() = default;

Eigen::VectorXd ConfigurationState::getQ() const
{
  Eigen::VectorXd q(19);
  q.segment<3>(0) = base_pos_;
  q.segment<4>(3) = base_quat_xyzw_;
  q.segment<3>(7) = FL_joint_angle_;
  q.segment<3>(10) = FR_joint_angle_;
  q.segment<3>(13) = RL_joint_angle_;
  q.segment<3>(16) = RR_joint_angle_;
  return q;
}

Eigen::VectorXd ConfigurationState::getDq() const
{
  Eigen::VectorXd dq(18);
  dq.segment<3>(0) = base_vel_;
  dq.segment<3>(3) = base_ang_vel_;
  dq.segment<3>(6) = FL_joint_vel_;
  dq.segment<3>(9) = FR_joint_vel_;
  dq.segment<3>(12) = RL_joint_vel_;
  dq.segment<3>(15) = RR_joint_vel_;
  return dq;
}

void ConfigurationState::updateQ(const Eigen::VectorXd & q)
{
  if (q.size() < 19) {
    throw std::invalid_argument("ConfigurationState::updateQ expects size 19");
  }
  base_pos_ = q.segment<3>(0);
  base_quat_xyzw_ = q.segment<4>(3);
  Eigen::VectorXd j = q.segment(7, 12);
  FL_joint_angle_ = j.segment<3>(0);
  FR_joint_angle_ = j.segment<3>(3);
  RL_joint_angle_ = j.segment<3>(6);
  RR_joint_angle_ = j.segment<3>(9);
}

void ConfigurationState::updateDq(const Eigen::VectorXd & dq)
{
  if (dq.size() < 18) {
    throw std::invalid_argument("ConfigurationState::updateDq expects size 18");
  }
  base_vel_ = dq.segment<3>(0);
  base_ang_vel_ = dq.segment<3>(3);
  Eigen::VectorXd jv = dq.segment(6, 12);
  FL_joint_vel_ = jv.segment<3>(0);
  FR_joint_vel_ = jv.segment<3>(3);
  RL_joint_vel_ = jv.segment<3>(6);
  RR_joint_vel_ = jv.segment<3>(9);
}

Eigen::Vector3d ConfigurationState::computeEulerAngleWorld() const
{
  const Eigen::Quaterniond q(
    base_quat_xyzw_[3], base_quat_xyzw_[0], base_quat_xyzw_[1], base_quat_xyzw_[2]);
  const Eigen::Matrix3d R = q.toRotationMatrix();

  const Eigen::Vector3d rpy = pinocchio::rpy::matrixToRpy(R);
  const double roll = rpy[0];
  const double pitch = rpy[1];
  const double yaw_meas = rpy[2];

  if (!yaw_unwrap_initialized_) {
    yaw_unwrap_initialized_ = true;
    yaw_prev_meas_ = yaw_meas;
    yaw_cont_ = yaw_meas;
  } else {
    const double yaw_delta =
      std::fmod(yaw_meas - yaw_prev_meas_ + M_PI, 2.0 * M_PI) - M_PI;
    yaw_cont_ += yaw_delta;
    yaw_prev_meas_ = yaw_meas;
  }

  return Eigen::Vector3d(roll, pitch, yaw_cont_);
}

void ConfigurationState::updateWithEulerAngles(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  const double qx = sr * cp * cy - cr * sp * sy;
  const double qy = cr * sp * cy + sr * cp * sy;
  const double qz = cr * cp * sy - sr * sp * cy;
  const double qw = cr * cp * cy + sr * sp * sy;

  base_quat_xyzw_ = Eigen::Vector4d(qx, qy, qz, qw);
}

}
