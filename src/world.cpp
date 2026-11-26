#include "lidar_sim/world.h"

#include <algorithm>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "lidar_sim/geometry/cone.h"
#include "lidar_sim/geometry/cylinder.h"
#include "lidar_sim/geometry/ellipsoid.h"
#include "lidar_sim/geometry/plane.h"
#include "lidar_sim/geometry/sphere.h"
#include "lidar_sim/math_utils.h"

namespace lidar_sim {

double World::castRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) const {
  double closest = kNoHit;
  for (const auto& primitive : primitives_) {
    closest = std::min(closest, primitive->intersect(origin, direction));
  }
  return closest;
}

namespace {

/// Prefixes parse errors with the object they come from, so bad entries are easy to find.
std::runtime_error objectError(std::size_t index, const YAML::Node& object,
                               const std::string& message) {
  std::string name = "objects[" + std::to_string(index) + "]";
  if (object["description"]) {
    name += " (" + object["description"].as<std::string>() + ")";
  }
  return std::runtime_error(name + ": " + message);
}

Eigen::Vector3d readVector3(const YAML::Node& node) {
  if (!node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("expected a list of 3 numbers");
  }
  return Eigen::Vector3d(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

Eigen::Vector3d readVector3(const YAML::Node& object, const std::string& key) {
  if (!object[key]) {
    throw std::runtime_error("missing '" + key + "'");
  }
  try {
    return readVector3(object[key]);
  } catch (const std::exception& e) {
    throw std::runtime_error("'" + key + "': " + e.what());
  }
}

double readPositive(const YAML::Node& object, const std::string& key) {
  if (!object[key]) {
    throw std::runtime_error("missing '" + key + "'");
  }
  const double value = object[key].as<double>();
  if (value <= 0.0) {
    throw std::runtime_error("'" + key + "' must be positive");
  }
  return value;
}

std::unique_ptr<Primitive> makePrimitive(const YAML::Node& object) {
  if (!object["type"]) {
    throw std::runtime_error("missing 'type'");
  }
  const std::string type = object["type"].as<std::string>();
  const Eigen::Vector3d center = readVector3(object, "center");
  const Eigen::Quaterniond orientation = object["angles"]
                                             ? quaternionFromRpy(readVector3(object, "angles"))
                                             : Eigen::Quaterniond::Identity();

  if (type == "plane") {
    return std::make_unique<Plane>(center, orientation);
  }
  if (type == "sphere") {
    return std::make_unique<Sphere>(center, orientation, readPositive(object, "radius"));
  }
  if (type == "cylinder") {
    return std::make_unique<Cylinder>(center, orientation, readPositive(object, "radius"));
  }
  if (type == "cone") {
    return std::make_unique<Cone>(center, orientation, readPositive(object, "slope"));
  }
  if (type == "ellipsoid") {
    return std::make_unique<Ellipsoid>(center, orientation, readVector3(object, "size"));
  }
  throw std::runtime_error("unknown type '" + type + "'");
}

void addPlaneConstraints(const YAML::Node& object, Primitive& primitive) {
  const YAML::Node constraints = object["plane_constraints"];
  if (!constraints) {
    return;
  }
  if (!constraints.IsSequence()) {
    throw std::runtime_error("'plane_constraints' must be a list");
  }
  for (const YAML::Node& constraint : constraints) {
    primitive.addPlaneConstraint(
        PlaneConstraint(readVector3(constraint, "normal"), readVector3(constraint, "point")));
  }
}

}  // namespace

World loadWorldFile(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Cannot read world file '" + path + "': " + e.what());
  }

  const YAML::Node objects = root["objects"];
  if (!objects || !objects.IsSequence()) {
    throw std::runtime_error("World file '" + path + "' must contain an 'objects' list");
  }

  World world;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    const YAML::Node object = objects[i];
    try {
      std::unique_ptr<Primitive> primitive = makePrimitive(object);
      addPlaneConstraints(object, *primitive);
      world.add(std::move(primitive));
    } catch (const std::exception& e) {
      throw objectError(i, object, e.what());
    }
  }
  return world;
}

}  // namespace lidar_sim
