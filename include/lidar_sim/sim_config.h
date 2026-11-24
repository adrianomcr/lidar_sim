#ifndef LIDAR_SIM_SIM_CONFIG_H
#define LIDAR_SIM_SIM_CONFIG_H

#include <string>
#include <vector>

#include <Eigen/Core>

#include "lidar_sim/lidar_scanner.h"
#include "lidar_sim/motion/drone_simulator.h"
#include "lidar_sim/motion/motion_simulator.h"

namespace lidar_sim {

/// RViz marker size and color stored in params.yaml. The viewer does not draw these; they are kept
/// so saving the file does not drop the values the ROS nodes read.
struct MarkerStyle {
  Eigen::Vector3d scale = Eigen::Vector3d(0.8, 0.8, 0.35);
  Eigen::Vector4d color = Eigen::Vector4d(0.3, 0.3, 1.0, 0.5);
};

/// One waypoint as written in params.yaml. `yaw_deg` is degrees; `DroneSimulator` wants radians.
struct WaypointSpec {
  double time = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double yaw_deg = 0.0;
};

/// Contents of config/params.yaml.
///
/// The same file feeds the ROS launch file and the standalone viewer. Elevations, mounts and
/// waypoint headings stay in degrees here, matching the file, and are converted when a simulator
/// config is built.
struct SimulationConfig {
  struct Lidar {
    std::string world_frame = "world";
    std::string lidar_frame = "lidar";
    double scan_rate = 10.0;
    int num_azimuth_steps = 1024;
    int num_rings = 64;
    double min_elevation_deg = -45.0;
    double max_elevation_deg = 45.0;
    double min_range = 1.0;
    double max_range = 100.0;
    double intensity = 60.0;
    int num_threads = 8;
    Eigen::Vector3d initial_position = Eigen::Vector3d(0.0, 0.0, 2.0);
    Eigen::Vector3d initial_rpy_deg = Eigen::Vector3d::Zero();
  } lidar;

  struct Motion {
    std::string world_frame = "world";
    std::string lidar_frame = "lidar";
    double update_rate = 50.0;
    double gravity = 9.81;
    double start_delay = 0.0;
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d lidar_mount_rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d trajectory_center = Eigen::Vector3d::Zero();
    double trajectory_radius = 1.0;
    double trajectory_linear_speed = 0.0;  ///< Tangential speed [m/s]. Divided by the radius for rad/s.
    Eigen::Vector3d trajectory_normal = Eigen::Vector3d::UnitZ();  ///< (0, 0, 1) is parallel to xy.
    double position_gain = 0.0;
    double velocity_gain = 0.0;
    MarkerStyle lidar_marker;
  } motion;

  struct Drone {
    std::string world_frame = "world";
    std::string lidar_frame = "lidar";
    std::string drone_frame = "drone";
    double update_rate = 50.0;
    double gravity = 9.81;
    double drag_coefficient = 0.0;
    double hover_duration = 0.0;
    double min_attitude_control_altitude = 0.0;
    Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d initial_rpy_deg = Eigen::Vector3d::Zero();
    Eigen::Vector3d lidar_mount_rpy_deg = Eigen::Vector3d::Zero();
    double cruise_speed = 1.0;
    double distance_gain = 1.0;
    double velocity_gain = 1.0;
    double attitude_gain = 1.0;
    double max_thrust_to_weight = 1.3;
    double max_yaw_rate = 0.3;
    std::vector<WaypointSpec> waypoints;
    MarkerStyle lidar_marker;
    MarkerStyle setpoint_marker;
  } drone;

  LidarConfig lidarConfig() const;
  MotionSimulator::Config motionConfig() const;
  DroneSimulator::Config droneConfig() const;
};

/// @throws std::runtime_error if the file is missing or a required value is absent or mistyped.
SimulationConfig loadSimulationConfig(const std::string& path);

/// Writes `config` as params.yaml. The file is replaced only after the new contents are complete.
/// @throws std::runtime_error if the path cannot be written.
void saveSimulationConfig(const std::string& path, const SimulationConfig& config);

}  // namespace lidar_sim

#endif  // LIDAR_SIM_SIM_CONFIG_H
