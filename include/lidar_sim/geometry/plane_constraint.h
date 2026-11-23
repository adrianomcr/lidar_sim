#ifndef LIDAR_SIM_GEOMETRY_PLANE_CONSTRAINT_H
#define LIDAR_SIM_GEOMETRY_PLANE_CONSTRAINT_H

#include <Eigen/Core>

namespace lidar_sim {

/// Half-space used to cut away part of a primitive.
///
/// A point p is kept when normal . (p - point) > 0, i.e. on the side the normal points to.
/// The constraint is expressed in the local frame of the primitive it belongs to.
class PlaneConstraint {
 public:
  /// @param normal Plane normal (does not need to be unit length).
  /// @param point  Any point on the plane.
  PlaneConstraint(const Eigen::Vector3d& normal, const Eigen::Vector3d& point);

  bool contains(const Eigen::Vector3d& point) const { return normal_.dot(point) - offset_ > 0.0; }

 private:
  Eigen::Vector3d normal_;
  double offset_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_PLANE_CONSTRAINT_H
