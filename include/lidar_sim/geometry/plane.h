#ifndef LIDAR_SIM_GEOMETRY_PLANE_H
#define LIDAR_SIM_GEOMETRY_PLANE_H

#include "lidar_sim/geometry/primitive.h"

namespace lidar_sim {

/// Infinite plane through the local origin with normal along the local z axis.
/// Use plane constraints to turn it into a finite patch.
class Plane : public Primitive {
 public:
  using Primitive::Primitive;

 protected:
  int intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                     double distances[2]) const override;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_PLANE_H
