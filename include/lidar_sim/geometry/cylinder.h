#ifndef LIDAR_SIM_GEOMETRY_CYLINDER_H
#define LIDAR_SIM_GEOMETRY_CYLINDER_H

#include "lidar_sim/geometry/primitive.h"

namespace lidar_sim {

/// Infinite cylinder whose axis is the local z axis.
/// Use plane constraints to bound its length.
class Cylinder : public Primitive {
 public:
  Cylinder(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation, double radius);

 protected:
  int intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                     double distances[2]) const override;

 private:
  double radius_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_CYLINDER_H
