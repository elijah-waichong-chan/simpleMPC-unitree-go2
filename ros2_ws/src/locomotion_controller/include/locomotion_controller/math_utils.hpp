#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace locomotion_mpc {

inline double clamp(double value, double lo, double hi)
{
  return std::min(std::max(value, lo), hi);
}

inline double wrapToPi(double angle)
{
  double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0) {
    wrapped += 2.0 * M_PI;
  }
  return wrapped - M_PI;
}

inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(),
       v.z(), 0.0, -v.x(),
      -v.y(), v.x(), 0.0;
  return m;
}

}  // namespace locomotion_mpc
