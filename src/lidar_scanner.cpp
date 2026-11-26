#include "lidar_sim/lidar_scanner.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>

namespace lidar_sim {

LidarScanner::LidarScanner(const LidarConfig& config) : config_(config) {
  if (config_.scan_rate <= 0.0) {
    throw std::invalid_argument("scan_rate must be positive");
  }
  if (config_.num_azimuth_steps < 1 || config_.num_rings < 1) {
    throw std::invalid_argument("num_azimuth_steps and num_rings must be at least 1");
  }
  if (config_.max_elevation < config_.min_elevation) {
    throw std::invalid_argument("max_elevation must not be smaller than min_elevation");
  }
  if (config_.min_range < 0.0 || config_.max_range <= config_.min_range) {
    throw std::invalid_argument("ranges must satisfy 0 <= min_range < max_range");
  }
  config_.num_threads = std::max(1, std::min(config_.num_threads, config_.num_rings));

  const int num_azimuth = config_.num_azimuth_steps;
  const int num_rings = config_.num_rings;
  const double scan_period = 1.0 / config_.scan_rate;
  const double elevation_step =
      num_rings > 1 ? (config_.max_elevation - config_.min_elevation) / (num_rings - 1) : 0.0;

  azimuths_.resize(num_azimuth);
  firing_times_.resize(num_azimuth);
  // Column spacing dt = T / N_a. Rings inside a column are spaced by dt / N_r, so ring 0 of
  // the next column follows the last ring of this one by one ring step.
  const double column_dt = scan_period / num_azimuth;
  ring_time_step_ = column_dt / num_rings;
  beam_directions_.resize(static_cast<std::size_t>(num_azimuth) * num_rings);
  for (int i = 0; i < num_azimuth; ++i) {
    const double azimuth = 2.0 * M_PI * i / num_azimuth;
    azimuths_[i] = static_cast<float>(azimuth);
    firing_times_[i] = column_dt * i;
    for (int ring = 0; ring < num_rings; ++ring) {
      const double elevation = config_.min_elevation + ring * elevation_step;
      // Clockwise sweep: the azimuth grows towards the sensor's -y axis.
      beam_directions_[i * num_rings + ring] =
          Eigen::Vector3d(std::cos(elevation) * std::cos(azimuth),
                          -std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
    }
  }
}

void LidarScanner::scan(const World& world, const Eigen::Vector3d& position,
                        const Eigen::Quaterniond& orientation,
                        std::vector<LidarReturn>& returns) const {
  returns.resize(beam_directions_.size());
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();

  // Each thread writes a disjoint set of rings, so the output needs no synchronization.
  const int num_threads = config_.num_threads;
  std::vector<std::thread> workers;
  workers.reserve(num_threads - 1);
  for (int t = 1; t < num_threads; ++t) {
    const int first = t * config_.num_rings / num_threads;
    const int last = (t + 1) * config_.num_rings / num_threads;
    workers.emplace_back(&LidarScanner::scanRings, this, std::cref(world), std::cref(position),
                         std::cref(rotation), first, last, std::ref(returns));
  }
  scanRings(world, position, rotation, 0, config_.num_rings / num_threads, returns);
  for (std::thread& worker : workers) {
    worker.join();
  }
}

void LidarScanner::scanRings(const World& world, const Eigen::Vector3d& position,
                             const Eigen::Matrix3d& rotation, int first_ring, int last_ring,
                             std::vector<LidarReturn>& returns) const {
  const int num_rings = config_.num_rings;
  for (int i = 0; i < config_.num_azimuth_steps; ++i) {
    for (int ring = first_ring; ring < last_ring; ++ring) {
      const std::size_t index = static_cast<std::size_t>(i) * num_rings + ring;
      const Eigen::Vector3d& beam = beam_directions_[index];
      const double distance = world.castRay(position, rotation * beam);

      LidarReturn& hit = returns[index];
      hit.valid = distance > config_.min_range && distance < config_.max_range;
      if (!hit.valid) {
        continue;
      }
      hit.point = distance * beam;
      hit.distance = static_cast<float>(distance);
      hit.azimuth = azimuths_[i];
      hit.intensity = config_.intensity;
      hit.ring = static_cast<std::uint16_t>(ring);
      hit.time = firing_times_[i] + ring * ring_time_step_;
    }
  }
}

}  // namespace lidar_sim
