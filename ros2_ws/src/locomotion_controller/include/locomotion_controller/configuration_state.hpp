#pragma once

#include <Eigen/Dense>

namespace locomotion_mpc {

class ConfigurationState {
public:
  ConfigurationState();

  Eigen::VectorXd getQ() const;
  Eigen::VectorXd getDq() const;

  void updateQ(const Eigen::VectorXd & q);
  void updateDq(const Eigen::VectorXd & dq);

  // Returns roll, pitch, and continuous yaw (unwraps internally).
  Eigen::Vector3d computeEulerAngleWorld() const;
  void updateWithEulerAngles(double roll, double pitch, double yaw);

  const Eigen::Vector3d & basePos() const { return base_pos_; }
  const Eigen::Vector4d & baseQuat() const { return base_quat_xyzw_; }
  const Eigen::Vector3d & baseVel() const { return base_vel_; }
  const Eigen::Vector3d & baseAngVel() const { return base_ang_vel_; }

  const Eigen::Vector3d & flJointAngle() const { return FL_joint_angle_; }
  const Eigen::Vector3d & frJointAngle() const { return FR_joint_angle_; }
  const Eigen::Vector3d & rlJointAngle() const { return RL_joint_angle_; }
  const Eigen::Vector3d & rrJointAngle() const { return RR_joint_angle_; }

  const Eigen::Vector3d & flJointVel() const { return FL_joint_vel_; }
  const Eigen::Vector3d & frJointVel() const { return FR_joint_vel_; }
  const Eigen::Vector3d & rlJointVel() const { return RL_joint_vel_; }
  const Eigen::Vector3d & rrJointVel() const { return RR_joint_vel_; }

private:
  Eigen::Vector3d base_pos_{0.0, 0.0, 0.27};
  Eigen::Vector4d base_quat_xyzw_{0.0, 0.0, 0.0, 1.0};
  Eigen::Vector3d FL_joint_angle_{0.0, 0.9, -1.8};
  Eigen::Vector3d FR_joint_angle_{0.0, 0.9, -1.8};
  Eigen::Vector3d RL_joint_angle_{0.0, 0.9, -1.8};
  Eigen::Vector3d RR_joint_angle_{0.0, 0.9, -1.8};

  Eigen::Vector3d base_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d base_ang_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d FL_joint_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d FR_joint_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d RL_joint_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d RR_joint_vel_{0.0, 0.0, 0.0};

  mutable bool yaw_unwrap_initialized_{false};
  mutable double yaw_prev_meas_{0.0};
  mutable double yaw_cont_{0.0};
};

}
