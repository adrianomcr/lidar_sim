#ifndef LIDAR_SIM_GEOMETRY_PRIMITIVE_H
#define LIDAR_SIM_GEOMETRY_PRIMITIVE_H

#include <limits>
#include <vector>

#include <Eigen/Geometry>

#include "lidar_sim/geometry/plane_constraint.h"

namespace lidar_sim {

/// Ray distance reported when a ray does not hit anything.
constexpr double kNoHit = std::numeric_limits<double>::infinity();

/// Base class of every geometric primitive the LiDAR can see.
///
/// Each primitive is described in its own local frame, placed in the world by a center and an
/// orientation. Rays are transformed into that frame, intersected with the canonical shape, and
/// the hits are filtered by the primitive's plane constraints (also given in the local frame).
class Primitive {
 public:
  /// @param center      Origin of the local frame, in world coordinates.
  /// @param orientation Rotation from the local frame to the world frame.
  Primitive(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation);
  virtual ~Primitive() = default;

  void addPlaneConstraint(const PlaneConstraint& constraint) { constraints_.push_back(constraint); }

  /// Distance along the ray to the closest visible hit, or kNoHit.
  /// @param origin    Ray origin in the world frame.
  /// @param direction Unit ray direction in the world frame.
  double intersect(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const;

 protected:
  /// Ray parameters where the local-frame ray meets the surface, in ascending order.
  /// Writes at most two values to `distances` and returns how many were written.
  virtual int intersectLocal(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction,
                             double distances[2]) const = 0;

  /// Real roots of a*t^2 + b*t + c = 0 in ascending order; returns how many were written.
  static int solveQuadratic(double a, double b, double c, double roots[2]);

 private:
  bool satisfiesConstraints(const Eigen::Vector3d& local_point) const;

  Eigen::Vector3d center_;
  Eigen::Matrix3d world_to_local_;
  std::vector<PlaneConstraint> constraints_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_GEOMETRY_PRIMITIVE_H
