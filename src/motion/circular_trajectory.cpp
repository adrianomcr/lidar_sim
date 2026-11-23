#include "lidar_sim/motion/circular_trajectory.h"

#include <cmath>
#include <stdexcept>

namespace lidar_sim {

CircularTrajectory::CircularTrajectory(const Config& config) : config_(config) {
  if (config_.first_axis.norm() == 0.0 || config_.second_axis.norm() == 0.0) {
    throw std::invalid_argument("Circle axes must be non-zero");
  }
  config_.first_axis.normalize();
  config_.second_axis.normalize();
  if (std::abs(config_.first_axis.dot(config_.second_axis)) > 1e-6) {
    throw std::invalid_argument("Circle axes must be orthogonal");
  }
}

Eigen::Vector3d CircularTrajectory::position(double t) const {
  const double phase = config_.angular_speed * t;
  return config_.center + config_.radius * (std::cos(phase) * config_.first_axis +
                                            std::sin(phase) * config_.second_axis);
}

Eigen::Vector3d CircularTrajectory::velocity(double t) const {
  const double phase = config_.angular_speed * t;
  return config_.radius * config_.angular_speed *
         (-std::sin(phase) * config_.first_axis + std::cos(phase) * config_.second_axis);
}

}  // namespace lidar_sim
