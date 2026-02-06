#pragma once

#include <Eigen/Dense>

namespace locomotion_mpc {

class SwingTrajectory {
public:
  SwingTrajectory() = default;
  SwingTrajectory(const Eigen::Vector3d & p0,
                  const Eigen::Vector3d & pf,
                  double t_swing,
                  double h_sw);

  void eval(double t,
            Eigen::Vector3d & p,
            Eigen::Vector3d & v,
            Eigen::Vector3d & a) const;

private:
  Eigen::Vector3d p0_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pf_{Eigen::Vector3d::Zero()};
  double T_{1.0};
  double h_sw_{0.0};
};

}  // namespace locomotion_mpc
