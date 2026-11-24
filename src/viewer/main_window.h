#ifndef LIDAR_SIM_VIEWER_MAIN_WINDOW_H
#define LIDAR_SIM_VIEWER_MAIN_WINDOW_H

#include <cstdint>
#include <memory>
#include <string>

#include <Eigen/Core>

#include <QImage>
#include <QMainWindow>
#include <QString>

#include "lidar_sim/sim_config.h"
#include "lidar_sim/world.h"
#include "simulation_session.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;
class QTableWidget;
class QTimer;

namespace lidar_sim {

class SceneView;

/// Desktop front end: edit params.yaml, choose a world file, and watch the scan.
class MainWindow : public QMainWindow {
 public:
  explicit MainWindow();
  ~MainWindow() override;

  void loadFiles(const QString& world_path, const QString& params_path);
  std::uint64_t scanRevision() const;
  int validPoints() const;
  QImage grabView();

 private:
  void buildUi();
  QDoubleSpinBox* addDouble(QFormLayout* form, const QString& label, double minimum, double maximum,
                            const QString& suffix);
  QSpinBox* addInt(QFormLayout* form, const QString& label, int minimum, int maximum,
                   const QString& suffix);
  void addVector(QFormLayout* form, const QString& label, QDoubleSpinBox*& x, QDoubleSpinBox*& y,
                 QDoubleSpinBox*& z, double minimum, double maximum, const QString& suffix);
  void browseWorld();
  void browseParams();
  void fillForm(const SimulationConfig& config);
  SimulationConfig readForm() const;
  MotionMode currentMode() const;
  void validate(const SimulationConfig& config) const;
  void applyWithWorld(const std::shared_ptr<const World>& world);
  void reloadWorld();
  void loadParams();
  void saveParams();
  void play();
  void pause();
  void frameView();
  void onTick();
  void markDirty();
  void setWaypoints(const std::vector<WaypointSpec>& waypoints);

  SceneView* view_ = nullptr;
  QTimer* timer_ = nullptr;
  QLineEdit* world_path_ = nullptr;
  QLineEdit* params_path_ = nullptr;
  QComboBox* motion_mode_ = nullptr;
  QPushButton* apply_button_ = nullptr;
  QPushButton* play_button_ = nullptr;
  QPushButton* pause_button_ = nullptr;
  QComboBox* speed_ = nullptr;
  QSlider* point_size_ = nullptr;
  QDoubleSpinBox* point_decay_ = nullptr;
  QTableWidget* waypoints_ = nullptr;

  QDoubleSpinBox* scan_rate_ = nullptr;
  QSpinBox* azimuth_steps_ = nullptr;
  QSpinBox* num_rings_ = nullptr;
  QDoubleSpinBox* min_elevation_ = nullptr;
  QDoubleSpinBox* max_elevation_ = nullptr;
  QDoubleSpinBox* min_range_ = nullptr;
  QDoubleSpinBox* max_range_ = nullptr;
  QDoubleSpinBox* intensity_ = nullptr;
  QSpinBox* num_threads_ = nullptr;
  QDoubleSpinBox* lidar_x_ = nullptr;
  QDoubleSpinBox* lidar_y_ = nullptr;
  QDoubleSpinBox* lidar_z_ = nullptr;
  QDoubleSpinBox* lidar_roll_ = nullptr;
  QDoubleSpinBox* lidar_pitch_ = nullptr;
  QDoubleSpinBox* lidar_yaw_ = nullptr;

  QDoubleSpinBox* motion_rate_ = nullptr;
  QDoubleSpinBox* motion_gravity_ = nullptr;
  QDoubleSpinBox* start_delay_ = nullptr;
  QDoubleSpinBox* motion_x_ = nullptr;
  QDoubleSpinBox* motion_y_ = nullptr;
  QDoubleSpinBox* motion_z_ = nullptr;
  QDoubleSpinBox* motion_roll_ = nullptr;
  QDoubleSpinBox* motion_pitch_ = nullptr;
  QDoubleSpinBox* motion_yaw_ = nullptr;
  QDoubleSpinBox* motion_mount_roll_ = nullptr;
  QDoubleSpinBox* motion_mount_pitch_ = nullptr;
  QDoubleSpinBox* motion_mount_yaw_ = nullptr;
  QDoubleSpinBox* circle_x_ = nullptr;
  QDoubleSpinBox* circle_y_ = nullptr;
  QDoubleSpinBox* circle_z_ = nullptr;
  QDoubleSpinBox* circle_radius_ = nullptr;
  QDoubleSpinBox* circle_linear_speed_ = nullptr;
  QDoubleSpinBox* circle_normal_x_ = nullptr;
  QDoubleSpinBox* circle_normal_y_ = nullptr;
  QDoubleSpinBox* circle_normal_z_ = nullptr;
  QDoubleSpinBox* position_gain_ = nullptr;
  QDoubleSpinBox* velocity_gain_ = nullptr;

  QDoubleSpinBox* drone_rate_ = nullptr;
  QDoubleSpinBox* drone_gravity_ = nullptr;
  QDoubleSpinBox* drag_ = nullptr;
  QDoubleSpinBox* hover_ = nullptr;
  QDoubleSpinBox* min_altitude_ = nullptr;
  QDoubleSpinBox* drone_x_ = nullptr;
  QDoubleSpinBox* drone_y_ = nullptr;
  QDoubleSpinBox* drone_z_ = nullptr;
  QDoubleSpinBox* drone_roll_ = nullptr;
  QDoubleSpinBox* drone_pitch_ = nullptr;
  QDoubleSpinBox* drone_yaw_ = nullptr;
  QDoubleSpinBox* drone_mount_roll_ = nullptr;
  QDoubleSpinBox* drone_mount_pitch_ = nullptr;
  QDoubleSpinBox* drone_mount_yaw_ = nullptr;
  QDoubleSpinBox* cruise_speed_ = nullptr;
  QDoubleSpinBox* distance_gain_ = nullptr;
  QDoubleSpinBox* drone_velocity_gain_ = nullptr;
  QDoubleSpinBox* attitude_gain_ = nullptr;
  QDoubleSpinBox* max_thrust_ = nullptr;
  QDoubleSpinBox* max_yaw_rate_ = nullptr;

  SimulationConfig loaded_config_;
  std::shared_ptr<const World> world_;
  SimulationSession session_;
  Eigen::Vector3d bounds_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bounds_max_ = Eigen::Vector3d::Ones();
  MotionMode running_mode_ = MotionMode::Drone;
  std::size_t object_count_ = 0;
  std::uint64_t point_revision_ = 0;
  std::uint64_t line_revision_ = 0;
  std::string last_error_;
  bool dirty_ = false;
  bool started_ = false;
  bool shader_reported_ = false;
  bool filling_form_ = false;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_VIEWER_MAIN_WINDOW_H
