#pragma once

#include <array>
#include <string>

#include <Eigen/Dense>

#include "locomotion_controller/gait.hpp"
#include "locomotion_controller/pinocchio_model.hpp"
#include "locomotion_controller/swing_trajectory.hpp"

namespace locomotion_mpc {

struct LegOutput {
  Eigen::Vector3d tau{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_now{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_now{Eigen::Vector3d::Zero()};
};

class LegController {
public:
  LegController();

  LegOutput computeLegTorque(const std::string & leg,
                             PinocchioModel & go2,
                             const Gait & gait,
                             const Eigen::Vector3d & contact_force,
                             double current_time);

  Eigen::VectorXd computeAllLegTorques(PinocchioModel & go2,
                                       const Gait & gait,
                                       const Eigen::VectorXd & contact_forces_world,
                                       double current_time);

private:
  enum LegIndex { FL = 0, FR = 1, RL = 2, RR = 3 };

  static LegIndex legIndexFromName(const std::string & leg);

  Eigen::Vector4i last_mask_{Eigen::Vector4i::Constant(2)};
  Eigen::Vector4i current_mask_{Eigen::Vector4i::Constant(2)};
  bool has_current_mask_{false};

  std::array<double, 4> takeoff_time_{};
  std::array<SwingTrajectory, 4> swing_traj_{};
  std::array<Eigen::Vector3d, 4> td_pos_{};
};

}  // namespace locomotion_mpc
