#include "lidar_sim/sim_config.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include "lidar_sim/math_utils.h"

namespace lidar_sim {
namespace {

std::string join(const std::string& path, const char* key) { return path + key; }

YAML::Node requireMap(const YAML::Node& parent, const char* key, const std::string& path) {
  const YAML::Node node = parent[key];
  if (!node || !node.IsMap()) {
    throw std::runtime_error("Missing '" + join(path, key) + "'");
  }
  return node;
}

double readDouble(const YAML::Node& parent, const char* key, const std::string& path) {
  const YAML::Node node = parent[key];
  const std::string name = join(path, key);
  if (!node) {
    throw std::runtime_error("Missing '" + name + "'");
  }
  try {
    return node.as<double>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error("'" + name + "' must be a number");
  }
}

int readInt(const YAML::Node& parent, const char* key, const std::string& path) {
  const YAML::Node node = parent[key];
  const std::string name = join(path, key);
  if (!node) {
    throw std::runtime_error("Missing '" + name + "'");
  }
  try {
    return node.as<int>();
  } catch (const YAML::Exception&) {
    try {
      return static_cast<int>(std::lround(node.as<double>()));
    } catch (const YAML::Exception&) {
      throw std::runtime_error("'" + name + "' must be an integer");
    }
  }
}

std::string readString(const YAML::Node& parent, const char* key, const std::string& path) {
  const YAML::Node node = parent[key];
  const std::string name = join(path, key);
  if (!node) {
    throw std::runtime_error("Missing '" + name + "'");
  }
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception&) {
    throw std::runtime_error("'" + name + "' must be a string");
  }
}

Eigen::VectorXd readVector(const YAML::Node& parent, const char* key, const std::string& path,
                           int size) {
  const YAML::Node node = parent[key];
  const std::string name = join(path, key);
  if (!node || !node.IsSequence() || static_cast<int>(node.size()) != size) {
    throw std::runtime_error("'" + name + "' must be a list of " + std::to_string(size) +
                             " numbers");
  }
  Eigen::VectorXd values(size);
  for (int i = 0; i < size; ++i) {
    try {
      values[i] = node[i].as<double>();
    } catch (const YAML::Exception&) {
      throw std::runtime_error("'" + name + "' must be a list of " + std::to_string(size) +
                               " numbers");
    }
  }
  return values;
}

Eigen::Vector3d readVector3(const YAML::Node& parent, const char* key, const std::string& path) {
  return readVector(parent, key, path, 3);
}

Eigen::Vector4d readVector4(const YAML::Node& parent, const char* key, const std::string& path) {
  return readVector(parent, key, path, 4);
}

MarkerStyle readMarker(const YAML::Node& parent, const char* key, const std::string& path) {
  const YAML::Node node = requireMap(parent, key, path);
  const std::string nested = join(path, key) + "/";
  MarkerStyle marker;
  marker.scale = readVector3(node, "scale", nested);
  marker.color = readVector4(node, "color", nested);
  return marker;
}

std::vector<WaypointSpec> readWaypoints(const YAML::Node& parent, const std::string& path) {
  const YAML::Node list = parent["waypoints"];
  if (!list || !list.IsSequence()) {
    throw std::runtime_error("'" + path + "waypoints' must be a list");
  }
  std::vector<WaypointSpec> waypoints;
  for (std::size_t i = 0; i < list.size(); ++i) {
    const YAML::Node entry = list[i];
    const std::string name = path + "waypoints[" + std::to_string(i) + "]/";
    if (!entry.IsMap()) {
      throw std::runtime_error("'" + name + "' must be {time, position, yaw_deg}");
    }
    WaypointSpec waypoint;
    waypoint.time = readDouble(entry, "time", name);
    waypoint.position = readVector3(entry, "position", name);
    waypoint.yaw_deg = readDouble(entry, "yaw_deg", name);
    waypoints.push_back(waypoint);
  }
  return waypoints;
}

Eigen::Quaterniond rpyDegrees(const Eigen::Vector3d& degrees) {
  return quaternionFromRpy(degToRad(degrees.x()), degToRad(degrees.y()), degToRad(degrees.z()));
}

std::string formatNumber(double value) {
  std::ostringstream out;
  out.setf(std::ios::fmtflags(0), std::ios::floatfield);
  out.precision(10);
  out << value;
  return out.str();
}

template <typename Derived>
std::string formatVector(const Eigen::MatrixBase<Derived>& values) {
  std::ostringstream out;
  out << "[";
  for (int i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << formatNumber(values(i));
  }
  out << "]";
  return out.str();
}

void writeMarker(std::ostream& out, const std::string& indent, const std::string& name,
                 const MarkerStyle& marker) {
  out << indent << name << ":\n";
  out << indent << "  scale: " << formatVector(marker.scale) << "\n";
  out << indent << "  color: " << formatVector(marker.color) << "\n";
}

bool hasKey(const YAML::Node& node, const char* key) {
  const YAML::Node child = node[key];
  return child && !child.IsNull();
}

// The circle used to be two axes plus an angular speed. A file from before that change still loads:
// linear_speed = angular_speed * radius, and the normal is first_axis × second_axis.
void readTrajectory(const YAML::Node& trajectory, SimulationConfig::Motion& motion) {
  const std::string path = "sim_motion/trajectory/";
  motion.trajectory_center = readVector3(trajectory, "center", path);
  motion.trajectory_radius = readDouble(trajectory, "radius", path);
  if (hasKey(trajectory, "linear_speed") && hasKey(trajectory, "normal")) {
    motion.trajectory_linear_speed = readDouble(trajectory, "linear_speed", path);
    motion.trajectory_normal = readVector3(trajectory, "normal", path);
    return;
  }
  if (hasKey(trajectory, "angular_speed") && hasKey(trajectory, "first_axis") &&
      hasKey(trajectory, "second_axis")) {
    const double angular_speed = readDouble(trajectory, "angular_speed", path);
    const Eigen::Vector3d first_axis = readVector3(trajectory, "first_axis", path);
    const Eigen::Vector3d second_axis = readVector3(trajectory, "second_axis", path);
    const Eigen::Vector3d normal = first_axis.cross(second_axis);
    if (normal.norm() == 0.0) {
      throw std::runtime_error(
          "sim_motion/trajectory first_axis and second_axis must not be parallel");
    }
    motion.trajectory_linear_speed = angular_speed * motion.trajectory_radius;
    motion.trajectory_normal = normal;
    return;
  }
  throw std::runtime_error(
      "sim_motion/trajectory needs linear_speed and normal. An older file can instead give "
      "angular_speed, first_axis and second_axis");
}

}  // namespace

LidarConfig SimulationConfig::lidarConfig() const {
  LidarConfig config;
  config.scan_rate = lidar.scan_rate;
  config.num_azimuth_steps = lidar.num_azimuth_steps;
  config.num_rings = lidar.num_rings;
  config.min_elevation = degToRad(lidar.min_elevation_deg);
  config.max_elevation = degToRad(lidar.max_elevation_deg);
  config.min_range = lidar.min_range;
  config.max_range = lidar.max_range;
  config.intensity = static_cast<float>(lidar.intensity);
  config.num_threads = lidar.num_threads;
  return config;
}

MotionSimulator::Config SimulationConfig::motionConfig() const {
  MotionSimulator::Config config;
  config.gravity = motion.gravity;
  config.start_delay = motion.start_delay;
  config.position_gain = motion.position_gain;
  config.velocity_gain = motion.velocity_gain;
  config.initial_position = motion.initial_position;
  config.initial_orientation = rpyDegrees(motion.initial_rpy_deg);
  config.lidar_mount = rpyDegrees(motion.lidar_mount_rpy_deg);
  config.trajectory.center = motion.trajectory_center;
  config.trajectory.radius = motion.trajectory_radius;
  config.trajectory.angular_speed =
      angularSpeedFromLinear(motion.trajectory_linear_speed, motion.trajectory_radius);
  circleAxesFromNormal(motion.trajectory_normal, config.trajectory.first_axis,
                       config.trajectory.second_axis);
  return config;
}

DroneSimulator::Config SimulationConfig::droneConfig() const {
  DroneSimulator::Config config;
  config.gravity = drone.gravity;
  config.drag_coefficient = drone.drag_coefficient;
  config.hover_duration = drone.hover_duration;
  config.min_attitude_control_altitude = drone.min_attitude_control_altitude;
  config.initial_position = drone.initial_position;
  config.initial_orientation = rpyDegrees(drone.initial_rpy_deg);
  config.lidar_mount = rpyDegrees(drone.lidar_mount_rpy_deg);
  config.controller.gravity = drone.gravity;
  config.controller.cruise_speed = drone.cruise_speed;
  config.controller.distance_gain = drone.distance_gain;
  config.controller.velocity_gain = drone.velocity_gain;
  config.controller.attitude_gain = drone.attitude_gain;
  config.controller.max_thrust_to_weight = drone.max_thrust_to_weight;
  config.controller.max_yaw_rate = drone.max_yaw_rate;
  config.waypoints.reserve(drone.waypoints.size());
  for (const WaypointSpec& spec : drone.waypoints) {
    Waypoint waypoint;
    waypoint.time = spec.time;
    waypoint.position = spec.position;
    waypoint.yaw = degToRad(spec.yaw_deg);
    config.waypoints.push_back(waypoint);
  }
  return config;
}

SimulationConfig loadSimulationConfig(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Cannot read parameters '" + path + "': " + e.what());
  }

  SimulationConfig config;
  try {
    const YAML::Node lidar = requireMap(root, "sim_lidar", "");
    config.lidar.world_frame = readString(lidar, "world_frame", "sim_lidar/");
    config.lidar.lidar_frame = readString(lidar, "lidar_frame", "sim_lidar/");
    config.lidar.scan_rate = readDouble(lidar, "scan_rate", "sim_lidar/");
    config.lidar.num_azimuth_steps = readInt(lidar, "num_azimuth_steps", "sim_lidar/");
    config.lidar.num_rings = readInt(lidar, "num_rings", "sim_lidar/");
    config.lidar.min_elevation_deg = readDouble(lidar, "min_elevation_deg", "sim_lidar/");
    config.lidar.max_elevation_deg = readDouble(lidar, "max_elevation_deg", "sim_lidar/");
    config.lidar.min_range = readDouble(lidar, "min_range", "sim_lidar/");
    config.lidar.max_range = readDouble(lidar, "max_range", "sim_lidar/");
    config.lidar.intensity = readDouble(lidar, "intensity", "sim_lidar/");
    config.lidar.num_threads = readInt(lidar, "num_threads", "sim_lidar/");
    config.lidar.initial_position = readVector3(lidar, "initial_position", "sim_lidar/");
    config.lidar.initial_rpy_deg = readVector3(lidar, "initial_rpy_deg", "sim_lidar/");

    const YAML::Node motion = requireMap(root, "sim_motion", "");
    config.motion.world_frame = readString(motion, "world_frame", "sim_motion/");
    config.motion.lidar_frame = readString(motion, "lidar_frame", "sim_motion/");
    config.motion.update_rate = readDouble(motion, "update_rate", "sim_motion/");
    config.motion.gravity = readDouble(motion, "gravity", "sim_motion/");
    config.motion.start_delay = readDouble(motion, "start_delay", "sim_motion/");
    config.motion.initial_position = readVector3(motion, "initial_position", "sim_motion/");
    config.motion.initial_rpy_deg = readVector3(motion, "initial_rpy_deg", "sim_motion/");
    config.motion.lidar_mount_rpy_deg = readVector3(motion, "lidar_mount_rpy_deg", "sim_motion/");
    const YAML::Node trajectory = requireMap(motion, "trajectory", "sim_motion/");
    readTrajectory(trajectory, config.motion);
    const YAML::Node tracking = requireMap(motion, "tracking", "sim_motion/");
    config.motion.position_gain = readDouble(tracking, "position_gain", "sim_motion/tracking/");
    config.motion.velocity_gain = readDouble(tracking, "velocity_gain", "sim_motion/tracking/");
    config.motion.lidar_marker = readMarker(motion, "lidar_marker", "sim_motion/");

    const YAML::Node drone = requireMap(root, "sim_drone", "");
    config.drone.world_frame = readString(drone, "world_frame", "sim_drone/");
    config.drone.lidar_frame = readString(drone, "lidar_frame", "sim_drone/");
    config.drone.drone_frame = readString(drone, "drone_frame", "sim_drone/");
    config.drone.update_rate = readDouble(drone, "update_rate", "sim_drone/");
    config.drone.gravity = readDouble(drone, "gravity", "sim_drone/");
    config.drone.drag_coefficient = readDouble(drone, "drag_coefficient", "sim_drone/");
    config.drone.hover_duration = readDouble(drone, "hover_duration", "sim_drone/");
    config.drone.min_attitude_control_altitude =
        readDouble(drone, "min_attitude_control_altitude", "sim_drone/");
    config.drone.initial_position = readVector3(drone, "initial_position", "sim_drone/");
    config.drone.initial_rpy_deg = readVector3(drone, "initial_rpy_deg", "sim_drone/");
    config.drone.lidar_mount_rpy_deg = readVector3(drone, "lidar_mount_rpy_deg", "sim_drone/");
    const YAML::Node controller = requireMap(drone, "controller", "sim_drone/");
    config.drone.cruise_speed = readDouble(controller, "cruise_speed", "sim_drone/controller/");
    config.drone.distance_gain = readDouble(controller, "distance_gain", "sim_drone/controller/");
    config.drone.velocity_gain = readDouble(controller, "velocity_gain", "sim_drone/controller/");
    config.drone.attitude_gain = readDouble(controller, "attitude_gain", "sim_drone/controller/");
    config.drone.max_thrust_to_weight =
        readDouble(controller, "max_thrust_to_weight", "sim_drone/controller/");
    config.drone.max_yaw_rate = readDouble(controller, "max_yaw_rate", "sim_drone/controller/");
    config.drone.waypoints = readWaypoints(drone, "sim_drone/");
    config.drone.lidar_marker = readMarker(drone, "lidar_marker", "sim_drone/");
    config.drone.setpoint_marker = readMarker(drone, "setpoint_marker", "sim_drone/");
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Invalid parameters '" + path + "': " + e.what());
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("Invalid parameters '" + path + "': " + e.what());
  }
  return config;
}

void saveSimulationConfig(const std::string& path, const SimulationConfig& config) {
  std::ostringstream out;
  out << "# Runtime parameters of the lidar_sim nodes and of lidar_sim_viewer.\n";
  out << "#\n";
  out << "# Each top-level key is the private namespace of one ROS node. Units are SI\n";
  out << "# (m, s, rad/s); parameters ending in _deg are in degrees. Orientations are\n";
  out << "# [roll, pitch, yaw] applied as Rz(yaw) * Ry(pitch) * Rx(roll).\n";
  out << "# The world file is chosen separately (launch argument `world`, or the viewer).\n\n";

  const SimulationConfig::Lidar& lidar = config.lidar;
  out << "sim_lidar:\n";
  out << "  world_frame: " << lidar.world_frame << "\n";
  out << "  lidar_frame: " << lidar.lidar_frame << "\n\n";
  out << "  scan_rate: " << formatNumber(lidar.scan_rate) << "\n";
  out << "  num_azimuth_steps: " << lidar.num_azimuth_steps << "\n";
  out << "  num_rings: " << lidar.num_rings << "\n";
  out << "  min_elevation_deg: " << formatNumber(lidar.min_elevation_deg) << "\n";
  out << "  max_elevation_deg: " << formatNumber(lidar.max_elevation_deg) << "\n";
  out << "  min_range: " << formatNumber(lidar.min_range) << "\n";
  out << "  max_range: " << formatNumber(lidar.max_range) << "\n";
  out << "  intensity: " << formatNumber(lidar.intensity) << "\n";
  out << "  num_threads: " << lidar.num_threads << "\n\n";
  out << "  initial_position: " << formatVector(lidar.initial_position) << "\n";
  out << "  initial_rpy_deg: " << formatVector(lidar.initial_rpy_deg) << "\n\n";

  const SimulationConfig::Motion& motion = config.motion;
  out << "sim_motion:\n";
  out << "  world_frame: " << motion.world_frame << "\n";
  out << "  lidar_frame: " << motion.lidar_frame << "\n";
  out << "  update_rate: " << formatNumber(motion.update_rate) << "\n";
  out << "  gravity: " << formatNumber(motion.gravity) << "\n\n";
  out << "  start_delay: " << formatNumber(motion.start_delay) << "\n";
  out << "  initial_position: " << formatVector(motion.initial_position) << "\n";
  out << "  initial_rpy_deg: " << formatVector(motion.initial_rpy_deg) << "\n";
  out << "  lidar_mount_rpy_deg: " << formatVector(motion.lidar_mount_rpy_deg) << "\n\n";
  out << "  trajectory:\n";
  out << "    center: " << formatVector(motion.trajectory_center) << "\n";
  out << "    radius: " << formatNumber(motion.trajectory_radius) << "\n";
  out << "    linear_speed: " << formatNumber(motion.trajectory_linear_speed) << "\n";
  out << "    normal: " << formatVector(motion.trajectory_normal) << "\n\n";
  out << "  tracking:\n";
  out << "    position_gain: " << formatNumber(motion.position_gain) << "\n";
  out << "    velocity_gain: " << formatNumber(motion.velocity_gain) << "\n\n";
  writeMarker(out, "  ", "lidar_marker", motion.lidar_marker);
  out << "\n";

  const SimulationConfig::Drone& drone = config.drone;
  out << "sim_drone:\n";
  out << "  world_frame: " << drone.world_frame << "\n";
  out << "  lidar_frame: " << drone.lidar_frame << "\n";
  out << "  drone_frame: " << drone.drone_frame << "\n";
  out << "  update_rate: " << formatNumber(drone.update_rate) << "\n";
  out << "  gravity: " << formatNumber(drone.gravity) << "\n";
  out << "  drag_coefficient: " << formatNumber(drone.drag_coefficient) << "\n\n";
  out << "  hover_duration: " << formatNumber(drone.hover_duration) << "\n";
  out << "  min_attitude_control_altitude: " << formatNumber(drone.min_attitude_control_altitude)
      << "\n";
  out << "  initial_position: " << formatVector(drone.initial_position) << "\n";
  out << "  initial_rpy_deg: " << formatVector(drone.initial_rpy_deg) << "\n";
  out << "  lidar_mount_rpy_deg: " << formatVector(drone.lidar_mount_rpy_deg) << "\n\n";
  out << "  controller:\n";
  out << "    cruise_speed: " << formatNumber(drone.cruise_speed) << "\n";
  out << "    distance_gain: " << formatNumber(drone.distance_gain) << "\n";
  out << "    velocity_gain: " << formatNumber(drone.velocity_gain) << "\n";
  out << "    attitude_gain: " << formatNumber(drone.attitude_gain) << "\n";
  out << "    max_thrust_to_weight: " << formatNumber(drone.max_thrust_to_weight) << "\n";
  out << "    max_yaw_rate: " << formatNumber(drone.max_yaw_rate) << "\n\n";
  out << "  waypoints:\n";
  for (const WaypointSpec& waypoint : drone.waypoints) {
    out << "    - {time: " << formatNumber(waypoint.time)
        << ", position: " << formatVector(waypoint.position)
        << ", yaw_deg: " << formatNumber(waypoint.yaw_deg) << "}\n";
  }
  out << "\n";
  writeMarker(out, "  ", "lidar_marker", drone.lidar_marker);
  writeMarker(out, "  ", "setpoint_marker", drone.setpoint_marker);

  const std::string temporary = path + ".tmp";
  {
    std::ofstream file(temporary);
    if (!file) {
      throw std::runtime_error("Cannot write '" + temporary + "'");
    }
    file << out.str();
    if (!file) {
      throw std::runtime_error("Cannot write '" + temporary + "'");
    }
  }
  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    std::remove(temporary.c_str());
    throw std::runtime_error("Cannot replace '" + path + "'");
  }
}

}  // namespace lidar_sim
