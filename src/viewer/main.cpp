#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <QApplication>
#include <QElapsedTimer>
#include <QSurfaceFormat>
#include <QTimer>

#include "lidar_sim/lidar_scanner.h"
#include "lidar_sim/math_utils.h"
#include "lidar_sim/motion/drone_simulator.h"
#include "lidar_sim/motion/motion_simulator.h"
#include "lidar_sim/sim_config.h"
#include "lidar_sim/world.h"
#include "main_window.h"
#include "scene_mesh.h"

namespace {

struct Options {
  std::string world;
  std::string params;
  std::string snapshot;
  bool check = false;
  bool help = false;
};

void printHelp() {
  std::cout
      << "Usage: lidar_sim_viewer [--world file] [--params file] [--check] [--snapshot file]\n"
      << "\n"
      << "  --world file      World YAML. Default: config/world.yaml\n"
      << "  --params file     Parameter YAML. Default: config/params.yaml\n"
      << "  --check           Load both files, tessellate the world, compute one scan and exit\n"
      << "  --snapshot file   Save a PNG of the first view and exit\n"
      << "                    (QT_QPA_PLATFORM=offscreen works when there is no display)\n";
}

std::string defaultFile(const std::string& name, const char* argv0) {
  namespace fs = std::filesystem;
  std::vector<fs::path> candidates;
  candidates.push_back(fs::current_path() / "config" / name);
  candidates.push_back(fs::path(LIDAR_SIM_SOURCE_DIR) / "config" / name);
  std::error_code error;
  fs::path executable = fs::read_symlink("/proc/self/exe", error);
  if (error) {
    executable = fs::absolute(argv0);
  }
  const fs::path directory = executable.parent_path();
  candidates.push_back(directory / "../share/lidar_sim/config" / name);
  candidates.push_back(directory / "config" / name);
  for (const fs::path& candidate : candidates) {
    if (fs::exists(candidate)) {
      return fs::absolute(candidate).string();
    }
  }
  return (fs::path(LIDAR_SIM_SOURCE_DIR) / "config" / name).string();
}

bool near(double a, double b) { return std::abs(a - b) <= 1e-5 * (1.0 + std::abs(a)); }

int runCheck(const std::string& world_path, const std::string& params_path) {
  bool ok = true;
  try {
    const lidar_sim::SimulationConfig config = lidar_sim::loadSimulationConfig(params_path);
    const std::string temporary = "/tmp/lidar_sim_params_roundtrip.yaml";
    lidar_sim::saveSimulationConfig(temporary, config);
    const lidar_sim::SimulationConfig again = lidar_sim::loadSimulationConfig(temporary);
    std::filesystem::remove(temporary);
    const auto check = [&](const char* name, double left, double right) {
      if (!near(left, right)) {
        std::cerr << "roundtrip " << name << ": " << left << " vs " << right << "\n";
        ok = false;
      }
    };
    check("scan_rate", config.lidar.scan_rate, again.lidar.scan_rate);
    check("min_elevation_deg", config.lidar.min_elevation_deg, again.lidar.min_elevation_deg);
    check("max_elevation_deg", config.lidar.max_elevation_deg, again.lidar.max_elevation_deg);
    check("cruise_speed", config.drone.cruise_speed, again.drone.cruise_speed);
    check("radius", config.motion.trajectory_radius, again.motion.trajectory_radius);
    check("marker_alpha", config.motion.lidar_marker.color.w(), again.motion.lidar_marker.color.w());
    if (config.lidar.num_rings != again.lidar.num_rings ||
        config.lidar.num_azimuth_steps != again.lidar.num_azimuth_steps ||
        config.drone.waypoints.size() != again.drone.waypoints.size()) {
      std::cerr << "roundtrip integer or waypoint count mismatch\n";
      ok = false;
    }
    if (!config.drone.waypoints.empty()) {
      check("waypoint_time", config.drone.waypoints.front().time, again.drone.waypoints.front().time);
      check("waypoint_yaw", config.drone.waypoints.front().yaw_deg,
            again.drone.waypoints.front().yaw_deg);
    }

    const lidar_sim::SceneMesh mesh = lidar_sim::buildSceneMesh(world_path);
    const std::size_t vertices = mesh.opaqueVertexCount() + mesh.planeVertexCount();
    const lidar_sim::World world = lidar_sim::loadWorldFile(world_path);
    lidar_sim::LidarScanner scanner(config.lidarConfig());
    std::vector<lidar_sim::LidarReturn> returns;
    const auto started = std::chrono::steady_clock::now();
    const Eigen::Quaterniond orientation =
        lidar_sim::quaternionFromRpy(lidar_sim::degToRad(config.lidar.initial_rpy_deg.x()),
                                     lidar_sim::degToRad(config.lidar.initial_rpy_deg.y()),
                                     lidar_sim::degToRad(config.lidar.initial_rpy_deg.z()));
    scanner.scan(world, config.lidar.initial_position, orientation, returns);
    const double scan_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count();
    int valid = 0;
    for (const lidar_sim::LidarReturn& hit : returns) {
      valid += hit.valid ? 1 : 0;
    }

    lidar_sim::MotionSimulator motion(config.motionConfig());
    lidar_sim::DroneSimulator drone(config.droneConfig());
    for (int step = 0; step < 5; ++step) {
      motion.step(0.02 * (step + 1), 0.02);
      drone.step(0.02 * (step + 1), 0.02);
    }

    std::cout << "vertices=" << vertices << " triangles="
              << (mesh.opaque_indices.size() + mesh.plane_indices.size()) / 3
              << " objects=" << world.size() << " valid_points=" << valid
              << " scan_ms=" << scan_ms << "\n";
    if (vertices < 100 || valid < 100) {
      std::cerr << "expected a drawn world and a non-empty scan\n";
      ok = false;
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  options.world = defaultFile("world.yaml", argv[0]);
  options.params = defaultFile("params.yaml", argv[0]);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        options.help = true;
        return std::string();
      }
      return std::string(argv[++i]);
    };
    if (arg == "--help" || arg == "-h") {
      options.help = true;
    } else if (arg == "--world") {
      options.world = need("--world");
    } else if (arg == "--params") {
      options.params = need("--params");
    } else if (arg == "--snapshot") {
      options.snapshot = need("--snapshot");
    } else if (arg == "--check") {
      options.check = true;
    } else {
      std::cerr << "unknown argument " << arg << "\n";
      options.help = true;
    }
  }
  if (options.help) {
    printHelp();
    return 2;
  }
  if (options.check) {
    return runCheck(options.world, options.params);
  }

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setSamples(4);
  QSurfaceFormat::setDefaultFormat(format);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
  QApplication app(argc, argv);
  app.setApplicationName("lidar_sim_viewer");
  app.setStyle("Fusion");

  lidar_sim::MainWindow window;
  window.resize(1280, 800);
  window.show();
  window.loadFiles(QString::fromStdString(options.world), QString::fromStdString(options.params));

  if (!options.snapshot.empty()) {
    QElapsedTimer elapsed;
    elapsed.start();
    auto* timer = new QTimer(&app);
    QObject::connect(timer, &QTimer::timeout, &app, [&] {
      if (window.scanRevision() > 0) {
        timer->stop();
        QTimer::singleShot(400, &app, [&] {
          const QImage image = window.grabView();
          const bool saved = !image.isNull() && image.save(QString::fromStdString(options.snapshot));
          if (!saved || window.validPoints() < 100) {
            std::cerr << "snapshot failed (points=" << window.validPoints() << ")\n";
            app.exit(1);
            return;
          }
          std::cout << "wrote " << options.snapshot << " (" << image.width() << "x" << image.height()
                    << "), points=" << window.validPoints() << "\n";
          app.exit(0);
        });
      } else if (elapsed.elapsed() > 30000) {
        std::cerr << "timed out waiting for the first scan\n";
        app.exit(1);
      }
    });
    timer->start(100);
  }
  return app.exec();
}
