#include "lidar_sim/geometry/cone.h"

#include <cmath>
#include <stdexcept>

namespace lidar_sim {

Cone::Cone(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation, double slope)
    : Primitive(center, orientation), slope_squared_(slope * slope) {
  if (!(slope > 0.0)) {
    throw std::invalid_argument("Cone slope must be positive");
  }
}

int Cone::intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                         double distances[2]) const {
  // (x + t dx)^2 + (y + t dy)^2 - k^2 (z + t dz)^2 = 0
  const double dx = direction.x();
  const double dy = direction.y();
  const double dz = direction.z();
  const double ox = origin.x();
  const double oy = origin.y();
  const double oz = origin.z();
  const double a = dx * dx + dy * dy - slope_squared_ * dz * dz;
  const double b = 2.0 * (ox * dx + oy * dy - slope_squared_ * oz * dz);
  const double c = ox * ox + oy * oy - slope_squared_ * oz * oz;

  // A ray parallel to a generatrix makes the quadratic degenerate.
  if (std::abs(a) < 1e-12) {
    if (std::abs(b) < 1e-12) {
      return 0;
    }
    distances[0] = -c / b;
    return 1;
  }
  return solveQuadratic(a, b, c, distances);
}

}  // namespace lidar_sim
