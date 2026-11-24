#include "simulation_session.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include "lidar_sim/math_utils.h"

namespace lidar_sim {
namespace {

void pushVertex(std::vector<float>& dst, const Eigen::Vector3d& point, float r, float g,
                float b) {
  dst.push_back(static_cast<float>(point.x()));
  dst.push_back(static_cast<float>(point.y()));
  dst.push_back(static_cast<float>(point.z()));
  dst.push_back(0.0f);
  dst.push_back(0.0f);
  dst.push_back(1.0f);
  dst.push_back(r);
  dst.push_back(g);
  dst.push_back(b);
}

void addLine(std::vector<float>& dst, const Eigen::Vector3d& from, const Eigen::Vector3d& to,
             float r, float g, float b) {
  pushVertex(dst, from, r, g, b);
  pushVertex(dst, to, r, g, b);
}

void addCross(std::vector<float>& dst, const Eigen::Vector3d& center, double size, float r,
              float g, float b) {
  addLine(dst, center - size * Eigen::Vector3d::UnitX(), center + size * Eigen::Vector3d::UnitX(),
          r, g, b);
  addLine(dst, center - size * Eigen::Vector3d::UnitY(), center + size * Eigen::Vector3d::UnitY(),
          r, g, b);
  addLine(dst, center - size * Eigen::Vector3d::UnitZ(), center + size * Eigen::Vector3d::UnitZ(),
          r, g, b);
}

void heightColor(float z, bool ready, float z_min, float z_max, float fade, float& r, float& g,
                 float& b) {
  // Blue at the lowest world-frame height seen so far, red at the highest.
  float t = 0.5f;
  if (ready && z_max > z_min + 1e-4f) {
    t = std::clamp((z - z_min) / (z_max - z_min), 0.0f, 1.0f);
  }
  if (t < 0.25f) {
    const float u = t / 0.25f;
    r = 0.15f;
    g = 0.25f + 0.55f * u;
    b = 0.95f;
  } else if (t < 0.5f) {
    const float u = (t - 0.25f) / 0.25f;
    r = 0.15f;
    g = 0.80f + 0.15f * u;
    b = 0.95f * (1.0f - u);
  } else if (t < 0.75f) {
    const float u = (t - 0.5f) / 0.25f;
    r = 0.15f + 0.80f * u;
    g = 0.95f;
    b = 0.05f;
  } else {
    const float u = (t - 0.75f) / 0.25f;
    r = 0.95f;
    g = 0.95f * (1.0f - u) + 0.15f * u;
    b = 0.05f * (1.0f - u);
  }
  r *= fade;
  g *= fade;
  b *= fade;
}

}  // namespace

SimulationSession::~SimulationSession() { stop(); }

void SimulationSession::start(const SimulationConfig& config, MotionMode mode,
                              std::shared_ptr<const World> world) {
  if (!world) {
    throw std::invalid_argument("Load a world file first");
  }
  // Construct before stopping the run that is already on screen. A bad edit then leaves it going.
  auto scanner = std::make_shared<const LidarScanner>(config.lidarConfig());
  std::unique_ptr<MotionSimulator> motion;
  std::unique_ptr<DroneSimulator> drone;
  std::unique_ptr<CircularTrajectory> circle;
  if (mode == MotionMode::Circle) {
    const MotionSimulator::Config motion_config = config.motionConfig();
    motion = std::make_unique<MotionSimulator>(motion_config);
    circle = std::make_unique<CircularTrajectory>(motion_config.trajectory);
  } else if (mode == MotionMode::Drone) {
    drone = std::make_unique<DroneSimulator>(config.droneConfig());
  }

  stop();
  config_ = config;
  mode_ = mode;
  world_ = std::move(world);
  scanner_ = std::move(scanner);
  motion_ = std::move(motion);
  drone_ = std::move(drone);
  circle_ = std::move(circle);
  sim_time_ = 0.0;
  trail_.clear();
  returns_.clear();
  scans_.clear();
  z_ready_ = false;
  applied_decay_ = -1.0;
  playing_ = false;
  reset_requested_ = false;
  stop_ = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    published_.scanning = false;
    published_.error.clear();
    published_.sim_time = 0.0;
    published_.valid_points = 0;
    published_.points.clear();
    published_.sensor_lines.clear();
    published_.path_lines.clear();
  }
  thread_ = std::thread([this] { threadMain(); });
}

void SimulationSession::stop() {
  stop_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  stop_ = false;
}

void SimulationSession::setPlaying(bool playing) { playing_ = playing; }

void SimulationSession::setSpeed(double speed) { speed_ = std::max(0.05, speed); }

void SimulationSession::setPointDecay(double seconds) { point_decay_ = std::max(0.0, seconds); }

void SimulationSession::resetTime() { reset_requested_ = true; }

SimulationSession::Snapshot SimulationSession::snapshotFor(std::uint64_t point_revision,
                                                           std::uint64_t line_revision) const {
  std::lock_guard<std::mutex> lock(mutex_);
  Snapshot snap;
  snap.sim_time = published_.sim_time;
  snap.scan_ms = published_.scan_ms;
  snap.valid_points = published_.valid_points;
  snap.scanning = published_.scanning;
  snap.error = published_.error;
  snap.point_revision = published_.point_revision;
  snap.line_revision = published_.line_revision;
  if (published_.point_revision != point_revision) {
    snap.points = published_.points;
  }
  if (published_.line_revision != line_revision) {
    snap.sensor_lines = published_.sensor_lines;
    snap.path_lines = published_.path_lines;
  }
  return snap;
}

std::uint64_t SimulationSession::pointRevision() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return published_.point_revision;
}

int SimulationSession::validPoints() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return published_.valid_points;
}

void SimulationSession::threadMain() {
  try {
    scan();
  } catch (const std::exception& e) {
    setError(e.what());
  }

  using Clock = std::chrono::steady_clock;
  double next_scan = 1.0 / std::max(config_.lidar.scan_rate, 1e-6);
  auto wall_anchor = Clock::now();
  double sim_anchor = 0.0;

  while (!stop_) {
    refreshCloudIfDecayChanged();
    if (reset_requested_.exchange(false)) {
      try {
        rebuildMotion();
        sim_time_ = 0.0;
        trail_.clear();
        scans_.clear();
        next_scan = 1.0 / std::max(config_.lidar.scan_rate, 1e-6);
        sim_anchor = 0.0;
        wall_anchor = Clock::now();
        scan();
      } catch (const std::exception& e) {
        setError(e.what());
        playing_ = false;
      }
    }

    if (!playing_ || mode_ == MotionMode::Static) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      wall_anchor = Clock::now();
      sim_anchor = sim_time_;
      continue;
    }

    const double speed = std::max(0.05, speed_.load());
    const double rate = mode_ == MotionMode::Circle ? config_.motion.update_rate
                                                    : config_.drone.update_rate;
    if (!(rate > 0.0)) {
      setError("update_rate must be positive");
      playing_ = false;
      continue;
    }
    const double dt = 1.0 / rate;
    const double scan_period = 1.0 / std::max(config_.lidar.scan_rate, 1e-6);
    const double budget = std::min(scan_period, 0.04 * speed);

    try {
      double advanced = 0.0;
      while (advanced + 1e-12 < budget && sim_time_ + 1e-12 < next_scan) {
        const double step = std::min(dt, std::min(budget - advanced, next_scan - sim_time_));
        sim_time_ += step;
        advanced += step;
        stepMotion(step);
      }
      publishLines();
      if (sim_time_ + 1e-9 >= next_scan) {
        scan();
        next_scan += scan_period;
      }
    } catch (const std::exception& e) {
      setError(e.what());
      playing_ = false;
      continue;
    }

    const double wall_due = (sim_time_ - sim_anchor) / speed;
    const auto deadline =
        wall_anchor + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(wall_due));
    while (!stop_ && !reset_requested_ && playing_ && Clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

void SimulationSession::rebuildMotion() {
  if (mode_ == MotionMode::Circle) {
    const MotionSimulator::Config motion_config = config_.motionConfig();
    motion_ = std::make_unique<MotionSimulator>(motion_config);
    circle_ = std::make_unique<CircularTrajectory>(motion_config.trajectory);
    drone_.reset();
  } else if (mode_ == MotionMode::Drone) {
    drone_ = std::make_unique<DroneSimulator>(config_.droneConfig());
    motion_.reset();
    circle_.reset();
  }
}

void SimulationSession::stepMotion(double dt) {
  if (mode_ == MotionMode::Circle) {
    motion_->step(sim_time_, dt);
  } else if (mode_ == MotionMode::Drone) {
    drone_->step(sim_time_, dt);
  }
  rememberTrail(sensorPosition());
}

void SimulationSession::rememberTrail(const Eigen::Vector3d& position) {
  if (!trail_.empty() && (trail_.back() - position).norm() < 0.05) {
    return;
  }
  trail_.push_back(position);
  if (trail_.size() > 8000) {
    trail_.erase(trail_.begin(), trail_.begin() + 2000);
  }
}

Eigen::Vector3d SimulationSession::sensorPosition() const {
  if (mode_ == MotionMode::Circle) {
    return motion_->state().position;
  }
  if (mode_ == MotionMode::Drone) {
    return drone_->state().position;
  }
  return config_.lidar.initial_position;
}

Eigen::Quaterniond SimulationSession::sensorOrientation() const {
  if (mode_ == MotionMode::Circle) {
    return motion_->imu().orientation;
  }
  if (mode_ == MotionMode::Drone) {
    return drone_->imu().orientation;
  }
  return quaternionFromRpy(degToRad(config_.lidar.initial_rpy_deg.x()),
                           degToRad(config_.lidar.initial_rpy_deg.y()),
                           degToRad(config_.lidar.initial_rpy_deg.z()));
}

void SimulationSession::storeScan(const Eigen::Vector3d& position,
                                  const Eigen::Quaterniond& orientation) {
  StoredScan stored;
  stored.time = sim_time_;
  stored.xyz.reserve(returns_.size() * 3);
  const Eigen::Matrix3d rotation = orientation.toRotationMatrix();
  for (const LidarReturn& hit : returns_) {
    if (!hit.valid) {
      continue;
    }
    const Eigen::Vector3d world = rotation * hit.point + position;
    const float z = static_cast<float>(world.z());
    if (!z_ready_) {
      z_min_ = z;
      z_max_ = z;
      z_ready_ = true;
    } else {
      z_min_ = std::min(z_min_, z);
      z_max_ = std::max(z_max_, z);
    }
    stored.xyz.push_back(static_cast<float>(world.x()));
    stored.xyz.push_back(static_cast<float>(world.y()));
    stored.xyz.push_back(z);
  }
  scans_.push_back(std::move(stored));
}

void SimulationSession::composePoints(std::vector<float>& points, int& count) {
  const double decay = std::max(0.0, point_decay_.load());
  const double now = sim_time_;
  if (decay <= 0.0) {
    while (scans_.size() > 1) {
      scans_.pop_front();
    }
  } else {
    while (!scans_.empty() && now - scans_.front().time >= decay) {
      scans_.pop_front();
    }
  }

  count = 0;
  for (const StoredScan& scan : scans_) {
    count += static_cast<int>(scan.xyz.size() / 3);
  }
  points.clear();
  points.reserve(static_cast<std::size_t>(count) * 9);
  for (const StoredScan& scan : scans_) {
    float fade = 1.0f;
    if (decay > 0.0) {
      const double age = std::max(0.0, now - scan.time);
      fade = static_cast<float>(std::clamp(1.0 - age / decay, 0.0, 1.0));
    }
    for (std::size_t i = 0; i + 2 < scan.xyz.size(); i += 3) {
      float red = 1.0f;
      float green = 1.0f;
      float blue = 1.0f;
      heightColor(scan.xyz[i + 2], z_ready_, z_min_, z_max_, fade, red, green, blue);
      points.push_back(scan.xyz[i]);
      points.push_back(scan.xyz[i + 1]);
      points.push_back(scan.xyz[i + 2]);
      points.push_back(0.0f);
      points.push_back(0.0f);
      points.push_back(1.0f);
      points.push_back(red);
      points.push_back(green);
      points.push_back(blue);
    }
  }
  applied_decay_ = decay;
}

void SimulationSession::refreshCloudIfDecayChanged() {
  const double decay = std::max(0.0, point_decay_.load());
  if (scans_.empty() || std::abs(decay - applied_decay_) <= 1e-9) {
    return;
  }
  std::vector<float> points;
  int count = 0;
  composePoints(points, count);
  std::lock_guard<std::mutex> lock(mutex_);
  published_.points.swap(points);
  published_.valid_points = count;
  ++published_.point_revision;
}

void SimulationSession::scan() {
  const auto started = std::chrono::steady_clock::now();
  const Eigen::Vector3d position = sensorPosition();
  const Eigen::Quaterniond orientation = sensorOrientation().normalized();
  scanner_->scan(*world_, position, orientation, returns_);
  const double scan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

  storeScan(position, orientation);
  std::vector<float> points;
  int count = 0;
  composePoints(points, count);
  rememberTrail(position);
  std::vector<float> sensor_lines = makeSensorLines(position, orientation);
  std::vector<float> path_lines = makePathLines(position);

  std::lock_guard<std::mutex> lock(mutex_);
  published_.points.swap(points);
  published_.sensor_lines.swap(sensor_lines);
  published_.path_lines.swap(path_lines);
  published_.sim_time = sim_time_;
  published_.scan_ms = scan_ms;
  published_.valid_points = count;
  published_.scanning = false;
  published_.error.clear();
  ++published_.point_revision;
  ++published_.line_revision;
}

void SimulationSession::publishLines() {
  const Eigen::Vector3d position = sensorPosition();
  const Eigen::Quaterniond orientation = sensorOrientation();
  std::vector<float> sensor_lines = makeSensorLines(position, orientation);
  std::vector<float> path_lines = makePathLines(position);
  std::lock_guard<std::mutex> lock(mutex_);
  published_.sensor_lines.swap(sensor_lines);
  published_.path_lines.swap(path_lines);
  published_.sim_time = sim_time_;
  published_.scanning = false;
  ++published_.line_revision;
}

void SimulationSession::setError(const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  published_.error = message;
  published_.scanning = false;
}

std::vector<float> SimulationSession::makeSensorLines(
    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) const {
  std::vector<float> lines;
  const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
  const double hx = 0.35;
  const double hy = 0.22;
  const double hz = 0.12;
  Eigen::Vector3d corners[8];
  int index = 0;
  for (double x : {-hx, hx}) {
    for (double y : {-hy, hy}) {
      for (double z : {-hz, hz}) {
        corners[index++] = position + rotation * Eigen::Vector3d(x, y, z);
      }
    }
  }
  const int edges[12][2] = {{0, 1}, {0, 2}, {0, 4}, {1, 3}, {1, 5}, {2, 3},
                            {2, 6}, {3, 7}, {4, 5}, {4, 6}, {5, 7}, {6, 7}};
  for (const auto& edge : edges) {
    addLine(lines, corners[edge[0]], corners[edge[1]], 0.82f, 0.86f, 0.90f);
  }
  addLine(lines, position, position + rotation.col(0) * 1.15, 0.90f, 0.25f, 0.22f);
  addLine(lines, position, position + rotation.col(1) * 1.15, 0.25f, 0.78f, 0.32f);
  addLine(lines, position, position + rotation.col(2) * 1.15, 0.28f, 0.52f, 0.95f);
  return lines;
}

std::vector<float> SimulationSession::makePathLines(const Eigen::Vector3d& position) const {
  std::vector<float> lines;
  for (std::size_t i = 1; i < trail_.size(); ++i) {
    addLine(lines, trail_[i - 1], trail_[i], 0.95f, 0.78f, 0.20f);
  }
  const double radius = config_.motion.trajectory_radius;
  const double linear_speed = config_.motion.trajectory_linear_speed;
  if (circle_ && std::abs(linear_speed) > 1e-6 && std::abs(radius) > 1e-9) {
    const double period = 2.0 * M_PI * std::abs(radius / linear_speed);
    Eigen::Vector3d previous = circle_->position(0.0);
    for (int i = 1; i <= 96; ++i) {
      const Eigen::Vector3d point = circle_->position(period * i / 96.0);
      addLine(lines, previous, point, 0.35f, 0.68f, 0.95f);
      previous = point;
    }
  }
  if (drone_) {
    const Waypoint& target = drone_->target();
    for (const WaypointSpec& waypoint : config_.drone.waypoints) {
      const bool active = std::abs(waypoint.time - target.time) < 1e-9;
      if (active) {
        addCross(lines, waypoint.position, 0.75, 1.0f, 0.86f, 0.25f);
      } else {
        addCross(lines, waypoint.position, 0.35, 0.95f, 0.38f, 0.30f);
      }
    }
    addLine(lines, position, target.position, 0.95f, 0.45f, 0.35f);
  }
  return lines;
}

}  // namespace lidar_sim
