#ifndef LIDAR_SIM_GEOMETRY_ELLIPSOID_H
#define LIDAR_SIM_GEOMETRY_ELLIPSOID_H

#include "lidar_sim/geometry/primitive.h"

namespace lidar_sim {

/// Ellipsoid centered at the local origin with semi-axes along the local x, y and z axes.
class Ellipsoid : public Primitive {
 public:
  Ellipsoid(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& semi_axes);

 protected:
  int intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                     double distances[2]) const override;

 private:
  Eigen::Vector3d inverse_squared_semi_axes_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_ELLIPSOID_H
