#pragma once

#include <Eigen/Dense>
#include <string>
#include <utility>

#include "locomotion_controller/pinocchio_model.hpp"
#include "locomotion_controller/swing_trajectory.hpp"

namespace locomotion_mpc {

class Gait {
public:
  Gait(double frequency_hz, double duty,
       double ground_offset = 0.02,
       double swing_height = 0.1);

  void setGroundOffset(double ground_offset) { ground_offset_ = ground_offset; }
  void setSwingHeight(double swing_height) { swing_height_ = swing_height; }
  double groundOffset() const { return ground_offset_; }
  double swingHeight() const { return swing_height_; }

  double gaitHz() const { return gait_hz_; }
  double duty() const { return gait_duty_; }
  double gaitPeriod() const { return gait_period_; }
  double stanceTime() const { return stance_time_; }
  double swingTime() const { return swing_time_; }

  Eigen::Vector4i computeCurrentMask(double time) const;
  Eigen::Matrix<int, 4, Eigen::Dynamic> computeContactTable(double t0,
                                                            double dt,
                                                            int N) const;

  Eigen::Vector3d computeTouchdownWorldForTrajPurposeOnly(const PinocchioModel & go2,
                                                          const std::string & leg) const;
  std::pair<SwingTrajectory, Eigen::Vector3d> computeSwingTrajAndTouchdown(
    const PinocchioModel & go2,
    const std::string & leg) const;

private:
  double gait_duty_{0.0};
  double gait_hz_{1.0};
  double gait_period_{1.0};
  double stance_time_{0.0};
  double swing_time_{0.0};
  double ground_offset_{0.02};
  double swing_height_{0.1};
};

}  // namespace locomotion_mpc
