#include "lidar_sim/geometry/plane_constraint.h"

#include <stdexcept>

namespace lidar_sim {

PlaneConstraint::PlaneConstraint(const Eigen::Vector3d& normal, const Eigen::Vector3d& point) {
  if (normal.norm() == 0.0) {
    throw std::invalid_argument("Plane constraint normal must be non-zero");
  }
  normal_ = normal.normalized();
  offset_ = normal_.dot(point);
}

}  // namespace lidar_sim
