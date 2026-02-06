#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "locomotion_controller/configuration_state.hpp"

namespace locomotion_mpc {

class PinocchioModel {
public:
  PinocchioModel(const std::string & urdf_path,
                 const std::vector<std::string> & package_dirs);

  void updateModel(const Eigen::VectorXd & q, const Eigen::VectorXd & dq);
  void updateModelSimplified(const Eigen::VectorXd & q, const Eigen::VectorXd & dq);

  Eigen::VectorXd computeComXVec();

  Eigen::Vector3d getHipOffset(const std::string & leg) const;

  std::array<Eigen::Vector3d, 4> getFootPlacementInWorld() const;
  std::array<Eigen::Vector3d, 4> getFootLeverWorld() const;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> getSingleFootStateInWorld(const std::string & leg) const;

  Eigen::Matrix<double, 3, 3> compute3x3FootJacobianWorld(const std::string & leg);
  Eigen::Matrix<double, 3, 3> compute3x3FootJacobianBody(const std::string & leg);
  Eigen::Matrix<double, 3, Eigen::Dynamic> computeFullFootJacobianWorld(const std::string & leg);
  Eigen::Vector3d computeJdotDqWorld(const std::string & leg);

  void computeDynamicsTerms(Eigen::VectorXd & g, Eigen::MatrixXd & C, Eigen::MatrixXd & M) const;

  const pinocchio::Model & model() const { return model_; }
  const pinocchio::Data & data() const { return data_; }
  const ConfigurationState & currentConfig() const { return current_config_; }

  const Eigen::Vector3d & posComWorld() const { return pos_com_world_; }
  const Eigen::Vector3d & velComWorld() const { return vel_com_world_; }
  const Eigen::Matrix3d & RBodyToWorld() const { return R_body_to_world_; }
  const Eigen::Matrix3d & RWorldToBody() const { return R_world_to_body_; }
  const Eigen::Matrix3d & Rz() const { return R_z_; }

  // Desired targets (set by controller)
  void setXPosDesWorld(double v) { x_pos_des_world_ = v; }
  void setYPosDesWorld(double v) { y_pos_des_world_ = v; }
  void setXVelDesWorld(double v) { x_vel_des_world_ = v; }
  void setYVelDesWorld(double v) { y_vel_des_world_ = v; }
  void setYawRateDesWorld(double v) { yaw_rate_des_world_ = v; }

  double xPosDesWorld() const { return x_pos_des_world_; }
  double yPosDesWorld() const { return y_pos_des_world_; }
  double xVelDesWorld() const { return x_vel_des_world_; }
  double yVelDesWorld() const { return y_vel_des_world_; }
  double yawRateDesWorld() const { return yaw_rate_des_world_; }

private:
  enum LegIndex { FL = 0, FR = 1, RL = 2, RR = 3 };

  static LegIndex legIndexFromName(const std::string & leg);

  void cacheIdsAndOffsets_();

  pinocchio::Model model_;
  pinocchio::Data data_;

  ConfigurationState current_config_;

  pinocchio::FrameIndex base_id_{0};
  std::array<pinocchio::FrameIndex, 4> foot_ids_{};
  std::array<pinocchio::FrameIndex, 4> hip_ids_{};
  std::array<Eigen::Vector3d, 4> hip_offsets_{};

  std::array<std::array<pinocchio::JointIndex, 3>, 4> leg_joint_ids_{};
  std::array<std::array<int, 3>, 4> leg_vcols_{};

  pinocchio::SE3 oMb_;
  pinocchio::SE3 oMf1_;
  pinocchio::SE3 oMf2_;
  pinocchio::SE3 oMf3_;
  pinocchio::SE3 oMf4_;

  Eigen::Vector3d pos_com_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_com_world_{Eigen::Vector3d::Zero()};

  Eigen::Matrix3d R_body_to_world_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_world_to_body_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d R_z_{Eigen::Matrix3d::Identity()};

  double x_pos_des_world_{0.0};
  double y_pos_des_world_{0.0};
  double x_vel_des_world_{0.0};
  double y_vel_des_world_{0.0};
  double yaw_rate_des_world_{0.0};
};

}  // namespace locomotion_mpc
