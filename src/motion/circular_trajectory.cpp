#include "lidar_sim/motion/circular_trajectory.h"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace lidar_sim {

void circleAxesFromNormal(const Eigen::Vector3d& normal, Eigen::Vector3d& first_axis,
                          Eigen::Vector3d& second_axis) {
  const double length = normal.norm();
  if (length == 0.0) {
    throw std::invalid_argument("Circle normal must be non-zero");
  }
  // A rotation maps +z to the normal and carries +x, +y onto the plane, so the pair stays
  // right-handed. FromTwoVectors(+z, +z) is the identity, which is the xy circle.
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal / length);
  first_axis = rotation * Eigen::Vector3d::UnitX();
  second_axis = rotation * Eigen::Vector3d::UnitY();
}

double angularSpeedFromLinear(double linear_speed, double radius) {
  if (!(std::abs(radius) > 1e-12)) {
    return 0.0;
  }
  return linear_speed / radius;
}

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
