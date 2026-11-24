#ifndef LIDAR_SIM_VIEWER_SCENE_MESH_H
#define LIDAR_SIM_VIEWER_SCENE_MESH_H

#include <string>
#include <vector>

#include <Eigen/Core>

namespace lidar_sim {

/// Triangle meshes used to draw a world file.
///
/// Vertices are packed as px, py, pz, nx, ny, nz, r, g, b. Planes are separate because the viewer
/// draws them translucent: a closed hangar would otherwise hide the sensor and the points inside.
struct SceneMesh {
  std::vector<float> opaque_vertices;
  std::vector<unsigned int> opaque_indices;
  std::vector<float> plane_vertices;
  std::vector<unsigned int> plane_indices;
  Eigen::Vector3d bounds_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d bounds_max = Eigen::Vector3d::Zero();

  std::size_t opaqueVertexCount() const { return opaque_vertices.size() / 9; }
  std::size_t planeVertexCount() const { return plane_vertices.size() / 9; }
};

/// Tessellates the primitives in a world file.
///
/// Planes and cylinders are infinite in the simulation. Here a plane is the part of it that covers
/// the rest of the scene, clipped by its plane constraints, and a cylinder is cut to the scene
/// bounds along its axis (constraints still remove the hidden part). The result is a picture of
/// the surfaces the rays can hit, not an extra solid.
/// @throws std::runtime_error if the file cannot be read or an object is malformed.
SceneMesh buildSceneMesh(const std::string& world_path);

}  // namespace lidar_sim

#endif  // LIDAR_SIM_VIEWER_SCENE_MESH_H
