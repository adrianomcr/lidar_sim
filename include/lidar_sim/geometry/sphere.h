#ifndef LIDAR_SIM_GEOMETRY_SPHERE_H
#define LIDAR_SIM_GEOMETRY_SPHERE_H

#include "lidar_sim/geometry/primitive.h"

namespace lidar_sim {

/// Sphere centered at the local origin. The orientation only matters for plane constraints.
class Sphere : public Primitive {
 public:
  Sphere(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation, double radius);

 protected:
  int intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                     double distances[2]) const override;

 private:
  double radius_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_SPHERE_H
