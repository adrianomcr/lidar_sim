#include "lidar_sim/geometry/primitive.h"

#include <algorithm>
#include <cmath>

namespace lidar_sim {

Primitive::Primitive(const Eigen::Vector3d& center, const Eigen::Quaterniond& orientation)
    : center_(center), world_to_local_(orientation.normalized().toRotationMatrix().transpose()) {}

double Primitive::intersect(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const {
  // Rigid transforms preserve distances, so ray parameters in the local frame are world distances.
  const Eigen::Vector3d local_origin = world_to_local_ * (origin - center_);
  const Eigen::Vector3d local_direction = world_to_local_ * direction;

  double distances[2];
  const int count = intersectLocal(local_origin, local_direction, distances);
  for (int i = 0; i < count; ++i) {
    if (distances[i] <= 0.0) {
      continue;
    }
    if (satisfiesConstraints(local_origin + distances[i] * local_direction)) {
      return distances[i];
    }
  }
  return kNoHit;
}

int Primitive::solveQuadratic(double a, double b, double c, double roots[2]) {
  if (std::abs(a) < 1e-12) {
    return 0;
  }
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return 0;
  }
  const double sqrt_discriminant = std::sqrt(discriminant);
  const double r1 = (-b - sqrt_discriminant) / (2.0 * a);
  const double r2 = (-b + sqrt_discriminant) / (2.0 * a);
  roots[0] = std::min(r1, r2);
  roots[1] = std::max(r1, r2);
  return 2;
}

bool Primitive::satisfiesConstraints(const Eigen::Vector3d& local_point) const {
  for (const PlaneConstraint& constraint : constraints_) {
    if (!constraint.contains(local_point)) {
      return false;
    }
  }
  return true;
}

}  // namespace lidar_sim
