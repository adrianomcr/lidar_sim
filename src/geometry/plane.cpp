#include "lidar_sim/geometry/plane.h"

#include <cmath>

namespace lidar_sim {

int Plane::intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                          double distances[2]) const {
  if (std::abs(direction.z()) < 1e-12) {
    return 0;
  }
  distances[0] = -origin.z() / direction.z();
  return 1;
}

}  // namespace lidar_sim
