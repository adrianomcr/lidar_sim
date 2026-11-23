#include "lidar_sim/geometry/sphere.h"

namespace lidar_sim {

Sphere::Sphere(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation, double radius)
    : Primitive(center, orientation), radius_(radius) {}

int Sphere::intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                           double distances[2]) const {
  // |origin + t * direction|^2 = radius^2
  return solveQuadratic(direction.squaredNorm(), 2.0 * direction.dot(origin),
                        origin.squaredNorm() - radius_ * radius_, distances);
}

}  // namespace lidar_sim
