#include "lidar_sim/geometry/ellipsoid.h"

#include <stdexcept>

namespace lidar_sim {

Ellipsoid::Ellipsoid(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation,
                     const Eigen::Vector3d& semi_axes)
    : Primitive(center, orientation) {
  if ((semi_axes.array() <= 0.0).any()) {
    throw std::invalid_argument("Ellipsoid semi-axes must be positive");
  }
  inverse_squared_semi_axes_ = semi_axes.array().square().inverse();
}

int Ellipsoid::intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                              double distances[2]) const {
  // sum_i (origin_i + t * direction_i)^2 / semi_axis_i^2 = 1
  const Eigen::Vector3d scaled_direction = inverse_squared_semi_axes_.cwiseProduct(direction);
  const double a = direction.dot(scaled_direction);
  const double b = 2.0 * origin.dot(scaled_direction);
  const double c = origin.dot(inverse_squared_semi_axes_.cwiseProduct(origin)) - 1.0;
  return solveQuadratic(a, b, c, distances);
}

}  // namespace lidar_sim
