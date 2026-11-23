#ifndef LIDAR_SIM_WORLD_H
#define LIDAR_SIM_WORLD_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "lidar_sim/geometry/primitive.h"

namespace lidar_sim {

/// Collection of primitives that make up the simulated scene.
class World {
 public:
  void add(std::unique_ptr<Primitive> primitive) { primitives_.push_back(std::move(primitive)); }

  /// Distance to the closest primitive along the ray, or kNoHit.
  /// @param origin    Ray origin in the world frame.
  /// @param direction Unit ray direction in the world frame.
  double castRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const;

  std::size_t size() const { return primitives_.size(); }

 private:
  std::vector<std::unique_ptr<Primitive>> primitives_;
};

/// Builds a world from a YAML world description (see config/world.yaml for the format).
/// @throws std::runtime_error if the file cannot be read or an object is malformed.
World loadWorldFile(const std::string& path);

}  // namespace lidar_sim

#endif  // LIDAR_SIM_WORLD_H
