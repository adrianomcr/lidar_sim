#ifndef LIDAR_SIM_LIDAR_SCANNER_H
#define LIDAR_SIM_LIDAR_SCANNER_H

#include <cstdint>
#include <vector>

#include <Eigen/Geometry>

#include "lidar_sim/world.h"

namespace lidar_sim {

/// Geometry and timing of a spinning multi-beam LiDAR.
struct LidarConfig {
  double scan_rate = 10.0;        ///< Revolutions per second [Hz].
  int num_azimuth_steps = 1800;   ///< Firings per revolution.
  int num_rings = 16;             ///< Number of beams stacked in elevation.
  double min_elevation = 0.0;     ///< Elevation of ring 0 [rad].
  double max_elevation = 0.0;     ///< Elevation of the last ring [rad].
  double min_range = 1.0;         ///< Hits closer than this are dropped [m].
  double max_range = 100.0;       ///< Hits farther than this are dropped [m].
  float intensity = 60.0f;        ///< Intensity assigned to every return.
  int num_threads = 1;            ///< Worker threads used to cast the rays.
};

/// One firing of one beam. Geometric fields are only meaningful when `valid` is true.
struct LidarReturn {
  bool valid = false;
  Eigen::Vector3d point;  ///< Hit point in the sensor frame [m].
  float distance = 0.0f;  ///< Range to the hit [m].
  float azimuth = 0.0f;   ///< Firing azimuth, clockwise from the sensor x axis, in [0, 2*pi) [rad].
  float intensity = 0.0f;
  std::uint16_t ring = 0;  ///< Beam index, 0 is the lowest beam.
  double time = 0.0;       ///< Firing time relative to the start of the revolution [s].
};

/// Simulates full LiDAR revolutions by casting one ray per beam and firing against a World.
///
/// The beams sweep clockwise when seen from above, like a Velodyne sensor. Returns are ordered by
/// azimuth step first and ring second: index = azimuth_step * num_rings + ring.
class LidarScanner {
 public:
  /// @throws std::invalid_argument if the configuration is inconsistent.
  explicit LidarScanner(const LidarConfig& config);

  /// Simulates one revolution with the sensor frozen at the given pose.
  /// @param position    Sensor position in the world frame.
  /// @param orientation Rotation from the sensor frame to the world frame.
  /// @param returns     Resized to num_azimuth_steps * num_rings and overwritten.
  void scan(const World& world, const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation, std::vector<LidarReturn>& returns) const;

  const LidarConfig& config() const { return config_; }

 private:
  void scanRings(const World& world, const Eigen::Vector3d& position,
                 const Eigen::Matrix3d& rotation, int first_ring, int last_ring,
                 std::vector<LidarReturn>& returns) const;

  LidarConfig config_;
  std::vector<Eigen::Vector3d> beam_directions_;  ///< Unit beam directions in the sensor frame.
  std::vector<float> azimuths_;                   ///< Azimuth of each azimuth step.
  std::vector<double> firing_times_;              ///< Firing time of each azimuth step.
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_LIDAR_SCANNER_H
