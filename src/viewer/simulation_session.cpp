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

void ringColor(int ring, int num_rings, float& r, float& g, float& b) {
  const float hue =
      num_rings <= 1 ? 0.0f : static_cast<float>(ring) / static_cast<float>(num_rings - 1);
  const float chroma = 0.85f;
  const float x = chroma * (1.0f - std::fabs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
  const float m = 1.0f - chroma;
  float red = 0.0f;
  float green = 0.0f;
  float blue = 0.0f;
  switch (static_cast<int>(hue * 6.0f) % 6) {
    case 0:
      red = chroma;
      green = x;
      break;
    case 1:
      red = x;
      green = chroma;
      break;
    case 2:
      green = chroma;
      blue = x;
      break;
    case 3:
      green = x;
      blue = chroma;
      break;
    case 4:
      red = x;
      blue = chroma;
      break;
    default:
      red = chroma;
      blue = x;
      break;
  }
  r = red + m;
  g = green + m;
  b = blue + m;
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
  playing_ = false;
  reset_requested_ = false;
  stop_ = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    published_.scanning = true;
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

void SimulationSession::resetTime() { reset_requested_ = true; }

SimulationSession::Snapshot SimulationSession::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return published_;
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
    if (reset_requested_.exchange(false)) {
      try {
        rebuildMotion();
        sim_time_ = 0.0;
        trail_.clear();
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

void SimulationSession::scan() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    published_.scanning = true;
  }
  const auto started = std::chrono::steady_clock::now();
  const Eigen::Vector3d position = sensorPosition();
  const Eigen::Quaterniond orientation = sensorOrientation().normalized();
  scanner_->scan(*world_, position, orientation, returns_);
  const double scan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();

  std::vector<float> points;
  points.reserve(returns_.size() * 9);
  int valid = 0;
  const int rings = std::max(1, config_.lidar.num_rings);
  for (const LidarReturn& hit : returns_) {
    if (!hit.valid) {
      continue;
    }
    ++valid;
    float red = 1.0f;
    float green = 1.0f;
    float blue = 1.0f;
    ringColor(static_cast<int>(hit.ring), rings, red, green, blue);
    pushVertex(points, orientation * hit.point + position, red, green, blue);
  }
  rememberTrail(position);
  std::vector<float> sensor_lines = makeSensorLines(position, orientation);
  std::vector<float> path_lines = makePathLines(position);

  std::lock_guard<std::mutex> lock(mutex_);
  published_.points.swap(points);
  published_.sensor_lines.swap(sensor_lines);
  published_.path_lines.swap(path_lines);
  published_.sim_time = sim_time_;
  published_.scan_ms = scan_ms;
  published_.valid_points = valid;
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
  if (circle_ && std::abs(config_.motion.trajectory_angular_speed) > 1e-6) {
    const double period =
        2.0 * M_PI / std::abs(config_.motion.trajectory_angular_speed);
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
