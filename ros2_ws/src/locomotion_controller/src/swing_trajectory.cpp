#include "locomotion_controller/swing_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace locomotion_mpc {

SwingTrajectory::SwingTrajectory(const Eigen::Vector3d & p0,
                                 const Eigen::Vector3d & pf,
                                 double t_swing,
                                 double h_sw)
  : p0_(p0), pf_(pf), T_(t_swing), h_sw_(h_sw)
{
}

void SwingTrajectory::eval(double t,
                           Eigen::Vector3d & p,
                           Eigen::Vector3d & v,
                           Eigen::Vector3d & a) const
{
  if (T_ <= 0.0) {
    p = p0_;
    v.setZero();
    a.setZero();
    return;
  }

  const double s = std::clamp(t / T_, 0.0, 1.0);
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double s4 = s3 * s;
  const double s5 = s4 * s;

  const double mj = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
  const double dmj = 30.0 * s2 - 60.0 * s3 + 30.0 * s4;
  const double d2mj = 60.0 * s - 180.0 * s2 + 120.0 * s3;

  const Eigen::Vector3d dp = pf_ - p0_;
  p = p0_ + dp * mj;
  v = dp * (dmj / T_);
  a = dp * (d2mj / (T_ * T_));

  if (h_sw_ != 0.0) {
    const double one_s = 1.0 - s;
    const double b = 64.0 * s3 * one_s * one_s * one_s;
    const double db = 192.0 * s2 * one_s * one_s * (1.0 - 2.0 * s);
    const double d2b = 192.0 * (
      2.0 * s * one_s * one_s * (1.0 - 2.0 * s)
      - 2.0 * s2 * one_s * (1.0 - 2.0 * s)
      - 2.0 * s2 * one_s * one_s
    );

    p.z() += h_sw_ * b;
    v.z() += h_sw_ * db / T_;
    a.z() += h_sw_ * d2b / (T_ * T_);
  }
}

}  // namespace locomotion_mpc
