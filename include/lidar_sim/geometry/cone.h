#ifndef LIDAR_SIM_GEOMETRY_CONE_H
#define LIDAR_SIM_GEOMETRY_CONE_H

#include "lidar_sim/geometry/primitive.h"

namespace lidar_sim {

/// Infinite double cone x^2 + y^2 = slope^2 * z^2, apex at the local origin, axis along local z.
/// The radius at height z is slope * |z|. Use plane constraints to keep one nappe or a finite length.
class Cone : public Primitive {
 public:
  /// @param slope Radial growth per unit along the axis. Must be positive. Its sign would not
  ///              change the surface, because the equation uses slope squared.
  /// @throws std::invalid_argument if `slope` is not positive.
  Cone(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation, double slope);

 protected:
  int intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                     double distances[2]) const override;

 private:
  double slope_squared_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_CONE_H
