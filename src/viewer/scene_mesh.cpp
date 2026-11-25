#include "scene_mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "lidar_sim/math_utils.h"

namespace lidar_sim {
namespace {

struct HalfSpace {
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double offset = 0.0;
};

struct DrawObject {
  std::string type;
  std::string description;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  double radius = 1.0;
  double slope = 1.0;
  Eigen::Vector3d size = Eigen::Vector3d::Ones();
  std::vector<HalfSpace> constraints;
};

struct Bounds {
  Eigen::Vector3d min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

  void add(const Eigen::Vector3d& point) {
    if (!point.allFinite()) {
      return;
    }
    min = min.cwiseMin(point);
    max = max.cwiseMax(point);
  }

  bool valid() const { return min.x() <= max.x(); }
};

struct MeshAccum {
  std::vector<float>* vertices = nullptr;
  std::vector<unsigned int>* indices = nullptr;
  Bounds* bounds = nullptr;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3f color = Eigen::Vector3f::Ones();

  int add(const Eigen::Vector3d& local, const Eigen::Vector3d& local_normal) {
    const Eigen::Vector3d point = rotation * local + center;
    Eigen::Vector3d normal = rotation * local_normal;
    const double length = normal.norm();
    if (length > 1e-12) {
      normal /= length;
    } else {
      normal = Eigen::Vector3d::UnitZ();
    }
    const auto index = static_cast<unsigned int>(vertices->size() / 9);
    vertices->push_back(static_cast<float>(point.x()));
    vertices->push_back(static_cast<float>(point.y()));
    vertices->push_back(static_cast<float>(point.z()));
    vertices->push_back(static_cast<float>(normal.x()));
    vertices->push_back(static_cast<float>(normal.y()));
    vertices->push_back(static_cast<float>(normal.z()));
    vertices->push_back(color.x());
    vertices->push_back(color.y());
    vertices->push_back(color.z());
    bounds->add(point);
    return static_cast<int>(index);
  }

  void triangle(int a, int b, int c) {
    if (a < 0 || b < 0 || c < 0) {
      return;
    }
    indices->push_back(static_cast<unsigned int>(a));
    indices->push_back(static_cast<unsigned int>(b));
    indices->push_back(static_cast<unsigned int>(c));
  }
};

Eigen::Vector3d readVector3(const YAML::Node& node, const std::string& key) {
  const YAML::Node value = node[key];
  if (!value || !value.IsSequence() || value.size() != 3) {
    throw std::runtime_error("'" + key + "' must be a list of 3 numbers");
  }
  return Eigen::Vector3d(value[0].as<double>(), value[1].as<double>(), value[2].as<double>());
}

double readPositive(const YAML::Node& node, const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("missing '" + key + "'");
  }
  const double value = node[key].as<double>();
  if (!(value > 0.0)) {
    throw std::runtime_error("'" + key + "' must be positive");
  }
  return value;
}

bool contains(const std::vector<HalfSpace>& constraints, const Eigen::Vector3d& point) {
  for (const HalfSpace& constraint : constraints) {
    if (constraint.normal.dot(point) < constraint.offset - 1e-6) {
      return false;
    }
  }
  return true;
}

Eigen::Vector3f hashColor(const std::string& text, const Eigen::Vector3f& base) {
  std::uint32_t hash = 2166136261u;
  for (unsigned char character : text) {
    hash ^= character;
    hash *= 16777619u;
  }
  const float mix = static_cast<float>(hash % 100) / 100.0f;
  return base + Eigen::Vector3f(0.12f * mix, 0.05f * (1.0f - mix), -0.04f * mix);
}

std::array<Eigen::Vector3d, 8> cornersOf(const Bounds& bounds) {
  const Eigen::Vector3d& lo = bounds.min;
  const Eigen::Vector3d& hi = bounds.max;
  return {Eigen::Vector3d(lo.x(), lo.y(), lo.z()), Eigen::Vector3d(hi.x(), lo.y(), lo.z()),
          Eigen::Vector3d(lo.x(), hi.y(), lo.z()), Eigen::Vector3d(hi.x(), hi.y(), lo.z()),
          Eigen::Vector3d(lo.x(), lo.y(), hi.z()), Eigen::Vector3d(hi.x(), lo.y(), hi.z()),
          Eigen::Vector3d(lo.x(), hi.y(), hi.z()), Eigen::Vector3d(hi.x(), hi.y(), hi.z())};
}

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

std::vector<Vec2> clipPolygon(const std::vector<Vec2>& polygon, double nx, double ny,
                              double offset) {
  std::vector<Vec2> clipped;
  if (polygon.empty()) {
    return clipped;
  }
  auto inside = [&](const Vec2& point) { return nx * point.x + ny * point.y >= offset - 1e-8; };
  auto crossing = [&](const Vec2& from, const Vec2& to) {
    const double from_side = nx * from.x + ny * from.y - offset;
    const double to_side = nx * to.x + ny * to.y - offset;
    const double span = from_side - to_side;
    const double t = std::abs(span) < 1e-12 ? 0.0 : from_side / span;
    return Vec2{from.x + t * (to.x - from.x), from.y + t * (to.y - from.y)};
  };
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Vec2& current = polygon[i];
    const Vec2& previous = polygon[(i + polygon.size() - 1) % polygon.size()];
    const bool current_inside = inside(current);
    const bool previous_inside = inside(previous);
    if (current_inside) {
      if (!previous_inside) {
        clipped.push_back(crossing(previous, current));
      }
      clipped.push_back(current);
    } else if (previous_inside) {
      clipped.push_back(crossing(previous, current));
    }
  }
  return clipped;
}

void addPlane(const DrawObject& object, const Bounds& scene, MeshAccum& mesh) {
  const Eigen::Matrix3d world_to_local = object.rotation.transpose();
  double xmin = std::numeric_limits<double>::infinity();
  double xmax = -xmin;
  double ymin = xmin;
  double ymax = -xmin;
  for (const Eigen::Vector3d& corner : cornersOf(scene)) {
    const Eigen::Vector3d local = world_to_local * (corner - object.center);
    xmin = std::min(xmin, local.x());
    xmax = std::max(xmax, local.x());
    ymin = std::min(ymin, local.y());
    ymax = std::max(ymax, local.y());
  }
  if (!std::isfinite(xmin) || xmax - xmin < 1e-3 || ymax - ymin < 1e-3) {
    xmin = -5.0;
    xmax = 5.0;
    ymin = -5.0;
    ymax = 5.0;
  }
  xmin -= 0.05;
  xmax += 0.05;
  ymin -= 0.05;
  ymax += 0.05;

  // Counter-clockwise in the local xy plane, so the normal points along +z.
  std::vector<Vec2> polygon = {{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}};
  for (const HalfSpace& constraint : object.constraints) {
    polygon = clipPolygon(polygon, constraint.normal.x(), constraint.normal.y(), constraint.offset);
    if (polygon.size() < 3) {
      return;
    }
  }

  std::vector<int> ids;
  ids.reserve(polygon.size());
  for (const Vec2& point : polygon) {
    ids.push_back(mesh.add(Eigen::Vector3d(point.x, point.y, 0.0), Eigen::Vector3d::UnitZ()));
  }
  for (std::size_t i = 1; i + 1 < ids.size(); ++i) {
    mesh.triangle(ids[0], static_cast<int>(ids[i]), static_cast<int>(ids[i + 1]));
  }
}

// Local-z interval of an axis-aligned tube or cone: the scene box, then tightened by constraints
// whose normal is along the local z axis. Oblique cuts are applied later, per vertex.
bool axisSpan(const DrawObject& object, const Bounds& scene, double& zmin, double& zmax) {
  const Eigen::Vector3d axis = object.rotation.col(2);
  zmin = -std::numeric_limits<double>::infinity();
  zmax = std::numeric_limits<double>::infinity();
  bool missed = false;
  for (int axis_index = 0; axis_index < 3; ++axis_index) {
    if (std::abs(axis[axis_index]) < 1e-9) {
      if (object.center[axis_index] < scene.min[axis_index] ||
          object.center[axis_index] > scene.max[axis_index]) {
        missed = true;
      }
      continue;
    }
    double enter = (scene.min[axis_index] - object.center[axis_index]) / axis[axis_index];
    double leave = (scene.max[axis_index] - object.center[axis_index]) / axis[axis_index];
    if (enter > leave) {
      std::swap(enter, leave);
    }
    zmin = std::max(zmin, enter);
    zmax = std::min(zmax, leave);
  }
  // A default segment keeps a shape visible when its axis misses the scene box.
  if (missed || !(zmin < zmax) || zmax - zmin < 1e-3) {
    zmin = -5.0;
    zmax = 5.0;
  }
  zmin -= 0.02;
  zmax += 0.02;

  for (const HalfSpace& constraint : object.constraints) {
    if (constraint.normal.head<2>().norm() > 1e-8 || std::abs(constraint.normal.z()) < 1e-12) {
      continue;
    }
    const double limit = constraint.offset / constraint.normal.z();
    if (constraint.normal.z() > 0.0) {
      zmin = std::max(zmin, limit);
    } else {
      zmax = std::min(zmax, limit);
    }
  }
  return zmax - zmin >= 1e-4;
}

void addCylinder(const DrawObject& object, const Bounds& scene, MeshAccum& mesh) {
  double zmin = 0.0;
  double zmax = 0.0;
  if (!axisSpan(object, scene, zmin, zmax)) {
    return;
  }

  const double length = zmax - zmin;
  const int slices = 28;
  const int stacks = std::max(8, std::min(64, static_cast<int>(std::ceil(length / 0.2))));
  std::vector<int> ids(static_cast<std::size_t>(stacks + 1) * slices, -1);
  auto at = [&](int stack, int slice) {
    return static_cast<std::size_t>(stack * slices + (slice % slices));
  };
  for (int stack = 0; stack <= stacks; ++stack) {
    const double z = zmin + length * stack / stacks;
    for (int slice = 0; slice < slices; ++slice) {
      const double theta = 2.0 * M_PI * slice / slices;
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const Eigen::Vector3d local(object.radius * c, object.radius * s, z);
      if (!contains(object.constraints, local)) {
        continue;
      }
      ids[at(stack, slice)] = mesh.add(local, Eigen::Vector3d(c, s, 0.0));
    }
  }
  for (int stack = 0; stack < stacks; ++stack) {
    for (int slice = 0; slice < slices; ++slice) {
      const int a = ids[at(stack, slice)];
      const int b = ids[at(stack, slice + 1)];
      const int c = ids[at(stack + 1, slice)];
      const int d = ids[at(stack + 1, slice + 1)];
      mesh.triangle(a, b, d);
      mesh.triangle(a, d, c);
    }
  }
}

void addCone(const DrawObject& object, const Bounds& scene, MeshAccum& mesh) {
  double zmin = 0.0;
  double zmax = 0.0;
  if (!axisSpan(object, scene, zmin, zmax)) {
    return;
  }

  const double slope = object.slope;
  const double slope_squared = slope * slope;
  const double length = zmax - zmin;
  const int slices = 28;
  const int stacks = std::max(8, std::min(64, static_cast<int>(std::ceil(length / 0.2))));
  std::vector<int> ids(static_cast<std::size_t>(stacks + 1) * slices, -1);
  auto at = [&](int stack, int slice) {
    return static_cast<std::size_t>(stack * slices + (slice % slices));
  };
  for (int stack = 0; stack <= stacks; ++stack) {
    const double z = zmin + length * stack / stacks;
    const double radius = slope * std::abs(z);
    if (radius < 1e-8) {
      const Eigen::Vector3d apex_normal(0.0, 0.0, z >= 0.0 ? 1.0 : -1.0);
      const int apex = mesh.add(Eigen::Vector3d(0.0, 0.0, z), apex_normal);
      for (int slice = 0; slice < slices; ++slice) {
        ids[at(stack, slice)] = apex;
      }
      continue;
    }
    for (int slice = 0; slice < slices; ++slice) {
      const double theta = 2.0 * M_PI * slice / slices;
      const double c = std::cos(theta);
      const double s = std::sin(theta);
      const Eigen::Vector3d local(radius * c, radius * s, z);
      if (!contains(object.constraints, local)) {
        continue;
      }
      ids[at(stack, slice)] = mesh.add(local, Eigen::Vector3d(local.x(), local.y(), -slope_squared * z));
    }
  }
  for (int stack = 0; stack < stacks; ++stack) {
    for (int slice = 0; slice < slices; ++slice) {
      const int a = ids[at(stack, slice)];
      const int b = ids[at(stack, slice + 1)];
      const int c = ids[at(stack + 1, slice)];
      const int d = ids[at(stack + 1, slice + 1)];
      mesh.triangle(a, b, d);
      mesh.triangle(a, d, c);
    }
  }
}

void addEllipsoid(const DrawObject& object, const Eigen::Vector3d& semi_axes, MeshAccum& mesh) {
  const int stacks = 18;
  const int slices = 32;
  std::vector<int> ids(static_cast<std::size_t>(stacks + 1) * (slices + 1), -1);
  auto at = [&](int stack, int slice) {
    return static_cast<std::size_t>(stack * (slices + 1) + slice);
  };
  for (int stack = 0; stack <= stacks; ++stack) {
    const double phi = M_PI * stack / stacks;
    const double sin_phi = std::sin(phi);
    const double cos_phi = std::cos(phi);
    for (int slice = 0; slice <= slices; ++slice) {
      const double theta = 2.0 * M_PI * slice / slices;
      const Eigen::Vector3d unit(sin_phi * std::cos(theta), sin_phi * std::sin(theta), cos_phi);
      const Eigen::Vector3d local = unit.cwiseProduct(semi_axes);
      if (!contains(object.constraints, local)) {
        continue;
      }
      Eigen::Vector3d normal = unit.cwiseQuotient(semi_axes);
      if (normal.norm() < 1e-12) {
        normal = unit;
      }
      ids[at(stack, slice)] = mesh.add(local, normal);
    }
  }
  for (int stack = 0; stack < stacks; ++stack) {
    for (int slice = 0; slice < slices; ++slice) {
      const int a = ids[at(stack, slice)];
      const int b = ids[at(stack, slice + 1)];
      const int c = ids[at(stack + 1, slice)];
      const int d = ids[at(stack + 1, slice + 1)];
      mesh.triangle(a, b, d);
      mesh.triangle(a, d, c);
    }
  }
}

void expandFiniteBounds(const std::vector<DrawObject>& objects, Bounds& bounds) {
  for (const DrawObject& object : objects) {
    bounds.add(object.center);
    if (object.type == "sphere") {
      bounds.add(object.center + Eigen::Vector3d::Constant(object.radius));
      bounds.add(object.center - Eigen::Vector3d::Constant(object.radius));
    } else if (object.type == "ellipsoid") {
      Eigen::Vector3d half = Eigen::Vector3d::Zero();
      for (int axis = 0; axis < 3; ++axis) {
        half += object.rotation.col(axis).cwiseAbs() * object.size[axis];
      }
      bounds.add(object.center + half);
      bounds.add(object.center - half);
    } else if (object.type == "cylinder") {
      const Eigen::Vector3d axis = object.rotation.col(2);
      const Eigen::Vector3d side = axis.unitOrthogonal();
      const Eigen::Vector3d other = axis.cross(side).normalized();
      bounds.add(object.center + object.radius * side);
      bounds.add(object.center - object.radius * side);
      bounds.add(object.center + object.radius * other);
      bounds.add(object.center - object.radius * other);
    }
  }
  if (!bounds.valid()) {
    bounds.add(Eigen::Vector3d(-5.0, -5.0, -1.0));
    bounds.add(Eigen::Vector3d(5.0, 5.0, 5.0));
  }
  const Eigen::Vector3d center = 0.5 * (bounds.min + bounds.max);
  for (int axis = 0; axis < 3; ++axis) {
    if (bounds.max[axis] - bounds.min[axis] < 1.0) {
      bounds.min[axis] = center[axis] - 5.0;
      bounds.max[axis] = center[axis] + 5.0;
    }
  }
}

std::runtime_error objectError(std::size_t index, const YAML::Node& object,
                               const std::string& message) {
  std::string name = "objects[" + std::to_string(index) + "]";
  if (object["description"]) {
    name += " (" + object["description"].as<std::string>() + ")";
  }
  return std::runtime_error(name + ": " + message);
}

}  // namespace

SceneMesh buildSceneMesh(const std::string& world_path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(world_path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Cannot read world file '" + world_path + "': " + e.what());
  }
  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsSequence()) {
    throw std::runtime_error("World file '" + world_path + "' must contain an 'objects' list");
  }

  std::vector<DrawObject> drawables;
  drawables.reserve(objects.size());
  for (std::size_t i = 0; i < objects.size(); ++i) {
    const YAML::Node node = objects[i];
    try {
      if (!node["type"]) {
        throw std::runtime_error("missing 'type'");
      }
      DrawObject object;
      object.type = node["type"].as<std::string>();
      if (node["description"]) {
        object.description = node["description"].as<std::string>();
      }
      object.center = readVector3(node, "center");
      if (node["angles"]) {
        const Eigen::Vector3d angles = readVector3(node, "angles");
        object.rotation = quaternionFromRpy(angles).normalized().toRotationMatrix();
      }
      if (object.type == "sphere" || object.type == "cylinder") {
        object.radius = readPositive(node, "radius");
      } else if (object.type == "cone") {
        object.slope = readPositive(node, "slope");
      } else if (object.type == "ellipsoid") {
        object.size = readVector3(node, "size");
        if ((object.size.array() <= 0.0).any()) {
          throw std::runtime_error("'size' must be positive");
        }
      } else if (object.type != "plane") {
        throw std::runtime_error("unknown type '" + object.type + "'");
      }
      if (node["plane_constraints"]) {
        const YAML::Node constraints = node["plane_constraints"];
        if (!constraints.IsSequence()) {
          throw std::runtime_error("'plane_constraints' must be a list");
        }
        for (const YAML::Node& constraint : constraints) {
          const Eigen::Vector3d normal = readVector3(constraint, "normal");
          const Eigen::Vector3d point = readVector3(constraint, "point");
          if (normal.norm() == 0.0) {
            throw std::runtime_error("constraint normal must be non-zero");
          }
          object.constraints.push_back(HalfSpace{normal, normal.dot(point)});
        }
      }
      drawables.push_back(std::move(object));
    } catch (const std::exception& e) {
      throw objectError(i, node, e.what());
    }
  }

  Bounds scene;
  expandFiniteBounds(drawables, scene);

  SceneMesh mesh;
  Bounds drawn;
  MeshAccum opaque{&mesh.opaque_vertices, &mesh.opaque_indices, &drawn};
  MeshAccum planes{&mesh.plane_vertices, &mesh.plane_indices, &drawn};
  // Display colors only; the world file does not store them. Planes are gray, or green-gray when
  // a constraint cuts them. Cylinders stay blue-gray and cones a muted purple. Spheres and
  // ellipsoids use a warm base that shifts a little with the object description.
  for (const DrawObject& object : drawables) {
    if (object.type == "plane") {
      planes.rotation = object.rotation;
      planes.center = object.center;
      planes.color = object.constraints.empty() ? Eigen::Vector3f(0.62f, 0.66f, 0.72f)
                                                 : Eigen::Vector3f(0.45f, 0.68f, 0.55f);
      addPlane(object, scene, planes);
    } else {
      opaque.rotation = object.rotation;
      opaque.center = object.center;
      if (object.type == "sphere") {
        opaque.color = hashColor(object.description, Eigen::Vector3f(0.82f, 0.48f, 0.28f));
        addEllipsoid(object, Eigen::Vector3d::Constant(object.radius), opaque);
      } else if (object.type == "ellipsoid") {
        opaque.color = hashColor(object.description, Eigen::Vector3f(0.74f, 0.58f, 0.34f));
        addEllipsoid(object, object.size, opaque);
      } else if (object.type == "cylinder") {
        opaque.color = hashColor(object.description, Eigen::Vector3f(0.42f, 0.52f, 0.60f));
        addCylinder(object, scene, opaque);
      } else if (object.type == "cone") {
        opaque.color = hashColor(object.description, Eigen::Vector3f(0.55f, 0.46f, 0.68f));
        addCone(object, scene, opaque);
      }
    }
  }

  if (!drawn.valid()) {
    drawn.add(Eigen::Vector3d::Zero());
    drawn.add(Eigen::Vector3d(0.0, 0.0, 1.0));
  }
  mesh.bounds_min = drawn.min;
  mesh.bounds_max = drawn.max;
  return mesh;
}

}  // namespace lidar_sim
