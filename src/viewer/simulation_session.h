#ifndef LIDAR_SIM_VIEWER_SIMULATION_SESSION_H
#define LIDAR_SIM_VIEWER_SIMULATION_SESSION_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include "lidar_sim/lidar_scanner.h"
#include "lidar_sim/motion/circular_trajectory.h"
#include "lidar_sim/motion/drone_simulator.h"
#include "lidar_sim/motion/motion_simulator.h"
#include "lidar_sim/sim_config.h"
#include "lidar_sim/world.h"

namespace lidar_sim {

enum class MotionMode { Static, Circle, Drone };

/// Runs the LiDAR and the selected motion model off the GUI thread.
///
/// The world and the scanner are immutable while a scan is in progress. The GUI only reads the
/// latest snapshot, so it stays responsive when a revolution takes longer than the scan period.
class SimulationSession {
 public:
  struct Snapshot {
    std::vector<float> points;
    std::vector<float> sensor_lines;
    std::vector<float> path_lines;
    double sim_time = 0.0;
    double scan_ms = 0.0;
    int valid_points = 0;
    bool scanning = false;
    std::string error;
    std::uint64_t point_revision = 0;
    std::uint64_t line_revision = 0;
  };

  SimulationSession() = default;
  ~SimulationSession();

  SimulationSession(const SimulationSession&) = delete;
  SimulationSession& operator=(const SimulationSession&) = delete;

  /// Builds the scanner and the motion model, then computes a scan at t = 0.
  /// The previous run is stopped only after construction succeeds.
  /// @throws std::invalid_argument when the configuration is inconsistent.
  void start(const SimulationConfig& config, MotionMode mode, std::shared_ptr<const World> world);

  void stop();
  void setPlaying(bool playing);
  void setSpeed(double speed);
  /// How long a return stays in the displayed cloud, in simulation seconds.
  /// Zero keeps only the latest revolution. Older returns fade, then drop.
  void setPointDecay(double seconds);
  void resetTime();
  bool playing() const { return playing_.load(); }

  /// Copies the latest numbers. The point and line buffers are copied only when their revision
  /// differs from the revision the caller already drew.
  Snapshot snapshotFor(std::uint64_t point_revision, std::uint64_t line_revision) const;
  std::uint64_t pointRevision() const;
  int validPoints() const;

 private:
  void threadMain();
  void rebuildMotion();
  void scan();
  void publishLines();
  void setError(const std::string& message);
  void storeScan(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation);
  void composePoints(std::vector<float>& points, int& count);
  void refreshCloudIfDecayChanged();
  void stepMotion(double dt);
  void rememberTrail(const Eigen::Vector3d& position);
  Eigen::Vector3d sensorPosition() const;
  Eigen::Quaterniond sensorOrientation() const;
  std::vector<float> makeSensorLines(const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation) const;
  std::vector<float> makePathLines(const Eigen::Vector3d& position) const;

  SimulationConfig config_;
  MotionMode mode_ = MotionMode::Static;
  std::shared_ptr<const World> world_;
  std::shared_ptr<const LidarScanner> scanner_;
  std::unique_ptr<MotionSimulator> motion_;
  std::unique_ptr<DroneSimulator> drone_;
  std::unique_ptr<CircularTrajectory> circle_;
  double sim_time_ = 0.0;
  std::vector<Eigen::Vector3d> trail_;
  std::vector<LidarReturn> returns_;

  struct StoredScan {
    double time = 0.0;
    std::vector<float> xyz;  ///< World-frame positions, three floats per return.
  };
  std::deque<StoredScan> scans_;
  bool z_ready_ = false;
  float z_min_ = 0.0f;  ///< World-frame height range used to color the cloud.
  float z_max_ = 0.0f;
  double applied_decay_ = -1.0;

  mutable std::mutex mutex_;
  Snapshot published_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> playing_{false};
  std::atomic<bool> reset_requested_{false};
  std::atomic<double> speed_{1.0};
  std::atomic<double> point_decay_{0.0};
  std::thread thread_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_VIEWER_SIMULATION_SESSION_H
