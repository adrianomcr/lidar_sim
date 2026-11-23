#include "lidar_sim/geometry/cylinder.h"

namespace lidar_sim {

Cylinder::Cylinder(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation,
                   double radius)
    : Primitive(center, orientation), radius_(radius) {}

int Cylinder::intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                             double distances[2]) const {
  // x(t)^2 + y(t)^2 = radius^2; the z component is free along the axis.
  const Eigen::Vector2d o = origin.head<2>();
  const Eigen::Vector2d d = direction.head<2>();
  return solveQuadratic(d.squaredNorm(), 2.0 * d.dot(o), o.squaredNorm() - radius_ * radius_,
                        distances);
}

}  // namespace lidar_sim
