#include "locomotion_controller/pinocchio_model.hpp"

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace locomotion_mpc {

PinocchioModel::PinocchioModel(const std::string & urdf_path,
                               const std::vector<std::string> & package_dirs)
{
  (void)package_dirs;
  pinocchio::urdf::buildModel(
    urdf_path, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);

  // Seed with default config
  updateModel(current_config_.getQ(), current_config_.getDq());

  base_id_ = model_.getFrameId("base");
  cacheIdsAndOffsets_();
}

void PinocchioModel::cacheIdsAndOffsets_()
{
  foot_ids_[FL] = model_.getFrameId("FL_foot_joint");
  foot_ids_[FR] = model_.getFrameId("FR_foot_joint");
  foot_ids_[RL] = model_.getFrameId("RL_foot_joint");
  foot_ids_[RR] = model_.getFrameId("RR_foot_joint");

  hip_ids_[FL] = model_.getFrameId("FL_thigh_joint");
  hip_ids_[FR] = model_.getFrameId("FR_thigh_joint");
  hip_ids_[RL] = model_.getFrameId("RL_thigh_joint");
  hip_ids_[RR] = model_.getFrameId("RR_thigh_joint");

  for (int i = 0; i < 4; ++i) {
    if (foot_ids_[i] == (pinocchio::FrameIndex)(-1)) {
      throw std::runtime_error("Foot frame id not found.");
    }
    if (hip_ids_[i] == (pinocchio::FrameIndex)(-1)) {
      throw std::runtime_error("Hip frame id not found.");
    }
  }

  const pinocchio::SE3 & oMb = data_.oMf[base_id_];
  for (int i = 0; i < 4; ++i) {
    const pinocchio::SE3 & oMh = data_.oMf[hip_ids_[i]];
    hip_offsets_[i] = oMb.actInv(oMh).translation();
  }

  auto cache_leg_joints = [&](LegIndex leg, const std::string & prefix) {
    leg_joint_ids_[leg][0] = model_.getJointId(prefix + "_hip_joint");
    leg_joint_ids_[leg][1] = model_.getJointId(prefix + "_thigh_joint");
    leg_joint_ids_[leg][2] = model_.getJointId(prefix + "_calf_joint");
    for (int j = 0; j < 3; ++j) {
      if (leg_joint_ids_[leg][j] == (pinocchio::JointIndex)(-1)) {
        throw std::runtime_error("Leg joint id not found for " + prefix);
      }
      leg_vcols_[leg][j] = model_.joints[leg_joint_ids_[leg][j]].idx_v();
    }
  };

  cache_leg_joints(FL, "FL");
  cache_leg_joints(FR, "FR");
  cache_leg_joints(RL, "RL");
  cache_leg_joints(RR, "RR");
}

void PinocchioModel::updateModel(const Eigen::VectorXd & q, const Eigen::VectorXd & dq)
{
  current_config_.updateQ(q);
  current_config_.updateDq(dq);

  pinocchio::forwardKinematics(model_, data_, q, dq);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeAllTerms(model_, data_, q, dq);
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, dq);
  pinocchio::ccrba(model_, data_, q, dq);
  pinocchio::centerOfMass(model_, data_, q, dq);

  oMb_ = data_.oMf[base_id_];
  oMf1_ = data_.oMf[foot_ids_[FL]];
  oMf2_ = data_.oMf[foot_ids_[FR]];
  oMf3_ = data_.oMf[foot_ids_[RL]];
  oMf4_ = data_.oMf[foot_ids_[RR]];
  pos_com_world_ = data_.com[0];
  vel_com_world_ = data_.vcom[0];

  const double yaw = current_config_.computeEulerAngleWorld()[2];
  R_body_to_world_ = oMb_.rotation();
  R_world_to_body_ = R_body_to_world_.transpose();

  R_z_ << std::cos(yaw), -std::sin(yaw), 0.0,
          std::sin(yaw),  std::cos(yaw), 0.0,
          0.0,            0.0,           1.0;
}

void PinocchioModel::updateModelSimplified(const Eigen::VectorXd & q, const Eigen::VectorXd & dq)
{
  if (q.size() < 6 || dq.size() < 6) {
    throw std::invalid_argument("updateModelSimplified expects q,dq size >= 6");
  }
  const double roll = q[3];
  const double pitch = q[4];
  const double yaw = q[5];

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

  Eigen::VectorXd q_full(19);
  q_full.segment<3>(0) = q.segment<3>(0);
  q_full.segment<4>(3) = Eigen::Vector4d(qx, qy, qz, qw);
  q_full.segment<12>(7).setZero();

  Eigen::VectorXd dq_full(18);
  dq_full.segment<6>(0) = dq.segment<6>(0);
  dq_full.segment<12>(6).setZero();

  updateModel(q_full, dq_full);
}

Eigen::VectorXd PinocchioModel::computeComXVec()
{
  const Eigen::Vector3d rpy = current_config_.computeEulerAngleWorld();
  const Eigen::Vector3d rpy_rate_body = current_config_.baseAngVel();
  const Eigen::Vector3d omega_world = R_body_to_world_ * rpy_rate_body;

  Eigen::VectorXd x_vec(12);
  x_vec.segment<3>(0) = pos_com_world_;
  x_vec.segment<3>(3) = rpy;
  x_vec.segment<3>(6) = vel_com_world_;
  x_vec.segment<3>(9) = omega_world;
  return x_vec;
}

PinocchioModel::LegIndex PinocchioModel::legIndexFromName(const std::string & leg)
{
  if (leg == "FL") return FL;
  if (leg == "FR") return FR;
  if (leg == "RL") return RL;
  if (leg == "RR") return RR;
  throw std::invalid_argument("Unknown leg name: " + leg);
}

Eigen::Vector3d PinocchioModel::getHipOffset(const std::string & leg) const
{
  return hip_offsets_[legIndexFromName(leg)];
}

std::array<Eigen::Vector3d, 4> PinocchioModel::getFootPlacementInWorld() const
{
  return {
    oMf1_.translation(),
    oMf2_.translation(),
    oMf3_.translation(),
    oMf4_.translation()
  };
}

std::array<Eigen::Vector3d, 4> PinocchioModel::getFootLeverWorld() const
{
  return {
    oMf1_.translation() - pos_com_world_,
    oMf2_.translation() - pos_com_world_,
    oMf3_.translation() - pos_com_world_,
    oMf4_.translation() - pos_com_world_
  };
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
PinocchioModel::getSingleFootStateInWorld(const std::string & leg) const
{
  const LegIndex idx = legIndexFromName(leg);
  const pinocchio::SE3 & oMf = data_.oMf[foot_ids_[idx]];
  const Eigen::Vector3d foot_pos_world = oMf.translation();

  const pinocchio::Motion v6 = pinocchio::getFrameVelocity(
    model_, data_, foot_ids_[idx], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
  const Eigen::Vector3d foot_vel_world = v6.linear();

  return {foot_pos_world, foot_vel_world};
}

Eigen::Matrix<double, 3, 3>
PinocchioModel::compute3x3FootJacobianWorld(const std::string & leg)
{
  const LegIndex idx = legIndexFromName(leg);
  pinocchio::Data::Matrix6x J(6, model_.nv);
  pinocchio::getFrameJacobian(
    model_, data_, foot_ids_[idx], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

  const Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos = J.topRows<3>();
  Eigen::Matrix<double, 3, 3> J_leg;
  for (int i = 0; i < 3; ++i) {
    J_leg.col(i) = J_pos.col(leg_vcols_[idx][i]);
  }
  return J_leg;
}

Eigen::Matrix<double, 3, 3>
PinocchioModel::compute3x3FootJacobianBody(const std::string & leg)
{
  const LegIndex idx = legIndexFromName(leg);
  pinocchio::Data::Matrix6x J(6, model_.nv);
  pinocchio::getFrameJacobian(
    model_, data_, foot_ids_[idx], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  const Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos = J.topRows<3>();
  const Eigen::Matrix3d R_wb = oMb_.rotation();
  const Eigen::Matrix<double, 3, Eigen::Dynamic> J_pos_body = R_wb.transpose() * J_pos;

  Eigen::Matrix<double, 3, 3> J_leg;
  for (int i = 0; i < 3; ++i) {
    J_leg.col(i) = J_pos_body.col(leg_vcols_[idx][i]);
  }
  return J_leg;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioModel::computeFullFootJacobianWorld(const std::string & leg)
{
  const LegIndex idx = legIndexFromName(leg);
  pinocchio::Data::Matrix6x J(6, model_.nv);
  pinocchio::getFrameJacobian(
    model_, data_, foot_ids_[idx], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
  return J.topRows<3>();
}

Eigen::Vector3d PinocchioModel::computeJdotDqWorld(const std::string & leg)
{
  const LegIndex idx = legIndexFromName(leg);
  pinocchio::Data::Matrix6x Jdot(6, model_.nv);
  pinocchio::getFrameJacobianTimeVariation(
    model_, data_, foot_ids_[idx], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
  const Eigen::VectorXd dq = current_config_.getDq();
  return Jdot.topRows<3>() * dq;
}

void PinocchioModel::computeDynamicsTerms(Eigen::VectorXd & g,
                                          Eigen::MatrixXd & C,
                                          Eigen::MatrixXd & M) const
{
  g = data_.g;
  C = data_.C;
  M = data_.M;
}

}  // namespace locomotion_mpc
