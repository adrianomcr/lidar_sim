#include "main_window.h"

#include <cmath>
#include <utility>

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QShortcut>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWheelEvent>

#include "lidar_sim/lidar_scanner.h"
#include "lidar_sim/motion/drone_simulator.h"
#include "lidar_sim/motion/motion_simulator.h"
#include "scene_mesh.h"
#include "scene_view.h"

namespace lidar_sim {
namespace {

class DoubleBox : public QDoubleSpinBox {
 public:
  using QDoubleSpinBox::QDoubleSpinBox;

 protected:
  void wheelEvent(QWheelEvent* event) override {
    if (!hasFocus()) {
      event->ignore();
    } else {
      QDoubleSpinBox::wheelEvent(event);
    }
  }
};

class IntBox : public QSpinBox {
 public:
  using QSpinBox::QSpinBox;

 protected:
  void wheelEvent(QWheelEvent* event) override {
    if (!hasFocus()) {
      event->ignore();
    } else {
      QSpinBox::wheelEvent(event);
    }
  }
};

QString numberText(double value) { return QString::number(value, 'g', 10); }

void showError(QWidget* parent, const std::exception& error) {
  QMessageBox::warning(parent, "lidar_sim", QString::fromStdString(error.what()));
}

}  // namespace

MainWindow::MainWindow() {
  setWindowTitle("lidar_sim");
  buildUi();
  timer_ = new QTimer(this);
  QObject::connect(timer_, &QTimer::timeout, this, [this] { onTick(); });
  timer_->start(33);
  statusBar()->showMessage("Open a world file and a parameter file.");
}

MainWindow::~MainWindow() {
  if (timer_ != nullptr) {
    timer_->stop();
  }
  session_.stop();
}

void MainWindow::buildUi() {
  view_ = new SceneView(this);
  view_->setSpaceHandler([this] {
    if (session_.playing()) {
      pause();
    } else {
      play();
    }
  });

  auto* toolbar = addToolBar("Simulation");
  toolbar->setMovable(false);
  apply_button_ = new QPushButton("Apply", this);
  auto* save_button = new QPushButton("Save", this);
  play_button_ = new QPushButton("Play", this);
  pause_button_ = new QPushButton("Pause", this);
  auto* reset_button = new QPushButton("Reset", this);
  auto* fit_button = new QPushButton("Fit", this);
  pause_button_->setEnabled(false);
  apply_button_->setToolTip("Send the form to the running view");
  save_button->setToolTip("Write the form to the parameter file, then apply it");
  play_button_->setToolTip("Play the circle or the drone mission (Space in the 3D view)");
  reset_button->setToolTip("Restart the running simulation at t = 0");
  fit_button->setToolTip("Frame the scene (F)");
  toolbar->addWidget(apply_button_);
  toolbar->addWidget(save_button);
  toolbar->addSeparator();
  toolbar->addWidget(play_button_);
  toolbar->addWidget(pause_button_);
  toolbar->addWidget(reset_button);
  toolbar->addWidget(fit_button);
  toolbar->addSeparator();

  speed_ = new QComboBox(toolbar);
  for (double value : {0.5, 1.0, 2.0, 5.0, 10.0, 20.0}) {
    speed_->addItem(QString::number(value) + "x", value);
  }
  speed_->setCurrentIndex(1);
  speed_->setToolTip("Playback speed");
  toolbar->addWidget(speed_);

  auto addToggle = [&](const QString& text, bool checked, auto slot) {
    auto* box = new QCheckBox(text, toolbar);
    box->setChecked(checked);
    QObject::connect(box, &QCheckBox::toggled, this, slot);
    toolbar->addWidget(box);
  };
  addToggle("World", true, [this](bool on) { view_->setShowWorld(on); });
  addToggle("Points", true, [this](bool on) { view_->setShowPoints(on); });
  addToggle("Sensor", true, [this](bool on) { view_->setShowSensor(on); });
  addToggle("Path", true, [this](bool on) { view_->setShowPath(on); });
  addToggle("Wire", false, [this](bool on) { view_->setWireframe(on); });

  point_size_ = new QSlider(Qt::Horizontal, toolbar);
  point_size_->setRange(1, 12);
  point_size_->setValue(3);
  point_size_->setFixedWidth(90);
  point_size_->setToolTip("Point size");
  toolbar->addWidget(point_size_);

  point_decay_ = new DoubleBox(toolbar);
  point_decay_->setRange(0.0, 600.0);
  point_decay_->setDecimals(2);
  point_decay_->setSingleStep(0.5);
  point_decay_->setValue(0.0);
  point_decay_->setSuffix(" s");
  point_decay_->setSpecialValueText("latest");
  point_decay_->setFixedWidth(88);
  point_decay_->setToolTip(
      "How long returns stay on screen, in simulation seconds. 0 (latest) keeps only the newest "
      "scan. Older points fade, then disappear. Pause freezes the cloud.");
  auto* decay_label = new QLabel("Decay", toolbar);
  decay_label->setToolTip(point_decay_->toolTip());
  toolbar->addWidget(decay_label);
  toolbar->addWidget(point_decay_);

  auto* panel = new QWidget(this);
  auto* panel_layout = new QVBoxLayout(panel);
  panel_layout->setContentsMargins(8, 8, 8, 8);
  auto* intro = new QLabel(
      "Runs without ROS. The parameter file is the same config/params.yaml the launch file loads. "
      "Large planes are drawn translucent so a closed scene stays visible.",
      panel);
  intro->setWordWrap(true);
  panel_layout->addWidget(intro);

  auto* files = new QGroupBox("Files", panel);
  auto* files_layout = new QVBoxLayout(files);
  auto fileRow = [&](const QString& label, QLineEdit*& edit, const QString& button_text,
                     auto on_button) {
    auto* row = new QHBoxLayout();
    edit = new QLineEdit(files);
    auto* button = new QPushButton(button_text, files);
    row->addWidget(new QLabel(label, files));
    row->addWidget(edit, 1);
    row->addWidget(button);
    files_layout->addLayout(row);
    QObject::connect(button, &QPushButton::clicked, this, on_button);
  };
  fileRow("World", world_path_, "Browse", [this] { browseWorld(); });
  fileRow("Params", params_path_, "Browse", [this] { browseParams(); });
  auto* reload_row = new QHBoxLayout();
  auto* reload_world = new QPushButton("Reload world", files);
  auto* reload_params = new QPushButton("Reload parameters", files);
  reload_row->addWidget(reload_world);
  reload_row->addWidget(reload_params);
  files_layout->addLayout(reload_row);
  panel_layout->addWidget(files);

  auto* lidar_box = new QGroupBox("LiDAR", panel);
  auto* lidar_form = new QFormLayout(lidar_box);
  scan_rate_ = addDouble(lidar_form, "Scan rate", 0.1, 100.0, " Hz");
  azimuth_steps_ = addInt(lidar_form, "Azimuth steps", 1, 20000, "");
  num_rings_ = addInt(lidar_form, "Rings", 1, 256, "");
  min_elevation_ = addDouble(lidar_form, "Min elevation", -89.0, 89.0, " deg");
  max_elevation_ = addDouble(lidar_form, "Max elevation", -89.0, 89.0, " deg");
  min_range_ = addDouble(lidar_form, "Min range", 0.0, 10000.0, " m");
  max_range_ = addDouble(lidar_form, "Max range", 0.1, 10000.0, " m");
  intensity_ = addDouble(lidar_form, "Intensity", 0.0, 255.0, "");
  num_threads_ = addInt(lidar_form, "Threads", 1, 256, "");
  addVector(lidar_form, "Initial position", lidar_x_, lidar_y_, lidar_z_, -10000.0, 10000.0, " m");
  addVector(lidar_form, "Initial rpy", lidar_roll_, lidar_pitch_, lidar_yaw_, -360.0, 360.0,
            " deg");
  panel_layout->addWidget(lidar_box);

  motion_mode_ = new QComboBox(panel);
  motion_mode_->addItem("Static pose", static_cast<int>(MotionMode::Static));
  motion_mode_->addItem("Circle", static_cast<int>(MotionMode::Circle));
  motion_mode_->addItem("Drone", static_cast<int>(MotionMode::Drone));
  motion_mode_->setCurrentIndex(2);
  auto* mode_row = new QHBoxLayout();
  mode_row->addWidget(new QLabel("Motion", panel));
  mode_row->addWidget(motion_mode_, 1);
  panel_layout->addLayout(mode_row);

  auto* stack = new QStackedWidget(panel);
  auto* static_page = new QLabel(
      "The sensor stays at the LiDAR initial pose above. Circle and Drone use their own initial "
      "pose and move the sensor when you press Play.",
      stack);
  static_page->setWordWrap(true);
  static_page->setAlignment(Qt::AlignTop);
  stack->addWidget(static_page);

  auto* circle_page = new QWidget(stack);
  auto* circle_form = new QFormLayout(circle_page);
  motion_rate_ = addDouble(circle_form, "Update rate", 1.0, 1000.0, " Hz");
  motion_gravity_ = addDouble(circle_form, "Gravity", 0.0, 50.0, " m/s²");
  start_delay_ = addDouble(circle_form, "Start delay", 0.0, 10000.0, " s");
  addVector(circle_form, "Initial position", motion_x_, motion_y_, motion_z_, -10000.0, 10000.0,
            " m");
  addVector(circle_form, "Initial rpy", motion_roll_, motion_pitch_, motion_yaw_, -360.0, 360.0,
            " deg");
  addVector(circle_form, "LiDAR mount rpy", motion_mount_roll_, motion_mount_pitch_,
            motion_mount_yaw_, -360.0, 360.0, " deg");
  addVector(circle_form, "Circle center", circle_x_, circle_y_, circle_z_, -10000.0, 10000.0, " m");
  circle_radius_ = addDouble(circle_form, "Radius", 0.0, 10000.0, " m");
  circle_linear_speed_ = addDouble(circle_form, "Linear speed", -500.0, 500.0, " m/s");
  circle_linear_speed_->setToolTip(
      "Tangential speed. Angular speed is this value divided by the radius, right-handed about "
      "the normal.");
  addVector(circle_form, "Normal", circle_normal_x_, circle_normal_y_, circle_normal_z_, -100.0,
            100.0, "");
  circle_normal_x_->setToolTip("Plane normal x. [0, 0, 1] is a circle parallel to xy.");
  circle_normal_y_->setToolTip("Plane normal y. [0, 0, 1] is a circle parallel to xy.");
  circle_normal_z_->setToolTip("Plane normal z. [0, 0, 1] is a circle parallel to xy.");
  position_gain_ = addDouble(circle_form, "Position gain", 0.0, 100.0, " 1/s");
  velocity_gain_ = addDouble(circle_form, "Velocity gain", 0.0, 100.0, " 1/s");
  stack->addWidget(circle_page);

  auto* drone_page = new QWidget(stack);
  auto* drone_layout = new QVBoxLayout(drone_page);
  auto* drone_form_widget = new QWidget(drone_page);
  auto* drone_form = new QFormLayout(drone_form_widget);
  drone_form->setContentsMargins(0, 0, 0, 0);
  drone_rate_ = addDouble(drone_form, "Update rate", 1.0, 1000.0, " Hz");
  drone_gravity_ = addDouble(drone_form, "Gravity", 0.0, 50.0, " m/s²");
  drag_ = addDouble(drone_form, "Drag coefficient", 0.0, 100.0, " 1/s");
  hover_ = addDouble(drone_form, "Hover duration", 0.0, 10000.0, " s");
  min_altitude_ = addDouble(drone_form, "Min attitude altitude", -1000.0, 10000.0, " m");
  addVector(drone_form, "Initial position", drone_x_, drone_y_, drone_z_, -10000.0, 10000.0, " m");
  addVector(drone_form, "Initial rpy", drone_roll_, drone_pitch_, drone_yaw_, -360.0, 360.0,
            " deg");
  addVector(drone_form, "LiDAR mount rpy", drone_mount_roll_, drone_mount_pitch_, drone_mount_yaw_,
            -360.0, 360.0, " deg");
  cruise_speed_ = addDouble(drone_form, "Cruise speed", 0.0, 100.0, " m/s");
  distance_gain_ = addDouble(drone_form, "Distance gain", 0.0, 100.0, " 1/m");
  drone_velocity_gain_ = addDouble(drone_form, "Velocity gain", 0.0, 100.0, " 1/s");
  attitude_gain_ = addDouble(drone_form, "Attitude gain", 0.0, 100.0, " 1/s");
  max_thrust_ = addDouble(drone_form, "Max thrust / weight", 0.1, 10.0, "");
  max_yaw_rate_ = addDouble(drone_form, "Max yaw rate", 0.0, 20.0, " rad/s");
  drone_layout->addWidget(drone_form_widget);

  waypoints_ = new QTableWidget(0, 5, drone_page);
  waypoints_->setHorizontalHeaderLabels({"Time [s]", "X [m]", "Y [m]", "Z [m]", "Yaw [deg]"});
  waypoints_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  waypoints_->verticalHeader()->setVisible(false);
  waypoints_->setSelectionBehavior(QAbstractItemView::SelectRows);
  waypoints_->setMinimumHeight(160);
  drone_layout->addWidget(waypoints_);
  auto* waypoint_buttons = new QHBoxLayout();
  auto* add_waypoint = new QPushButton("Add waypoint", drone_page);
  auto* remove_waypoint = new QPushButton("Remove waypoint", drone_page);
  waypoint_buttons->addWidget(add_waypoint);
  waypoint_buttons->addWidget(remove_waypoint);
  drone_layout->addLayout(waypoint_buttons);
  stack->addWidget(drone_page);
  stack->setCurrentIndex(2);
  panel_layout->addWidget(stack);
  panel_layout->addStretch(1);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(panel);
  scroll->setMinimumWidth(340);

  auto* splitter = new QSplitter(this);
  splitter->addWidget(scroll);
  splitter->addWidget(view_);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({390, 900});
  setCentralWidget(splitter);

  QObject::connect(apply_button_, &QPushButton::clicked, this, [this] {
    try {
      applyWithWorld(world_);
    } catch (const std::exception& error) {
      showError(this, error);
    }
  });
  QObject::connect(save_button, &QPushButton::clicked, this, [this] { saveParams(); });
  QObject::connect(play_button_, &QPushButton::clicked, this, [this] { play(); });
  QObject::connect(pause_button_, &QPushButton::clicked, this, [this] { pause(); });
  QObject::connect(reset_button, &QPushButton::clicked, this, [this] { session_.resetTime(); });
  QObject::connect(fit_button, &QPushButton::clicked, this, [this] { frameView(); });
  QObject::connect(reload_world, &QPushButton::clicked, this, [this] { reloadWorld(); });
  QObject::connect(reload_params, &QPushButton::clicked, this, [this] { loadParams(); });
  QObject::connect(speed_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
    session_.setSpeed(speed_->currentData().toDouble());
  });
  QObject::connect(point_size_, &QSlider::valueChanged, this, [this](int value) {
    view_->setPointSize(static_cast<float>(value));
  });
  QObject::connect(point_decay_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                   [this](double value) { session_.setPointDecay(value); });
  QObject::connect(motion_mode_, qOverload<int>(&QComboBox::currentIndexChanged), this,
                   [this, stack](int index) {
                     stack->setCurrentIndex(index);
                     markDirty();
                   });
  QObject::connect(waypoints_, &QTableWidget::itemChanged, this, [this] { markDirty(); });
  QObject::connect(add_waypoint, &QPushButton::clicked, this, [this] {
    const int row = waypoints_->rowCount();
    waypoints_->insertRow(row);
    double values[5] = {0.0, 0.0, 0.0, 2.0, 0.0};
    if (row > 0) {
      for (int col = 0; col < 5; ++col) {
        QTableWidgetItem* item = waypoints_->item(row - 1, col);
        if (item != nullptr) {
          bool ok = false;
          const double parsed = item->text().toDouble(&ok);
          if (ok) {
            values[col] = parsed;
          }
        }
      }
      values[0] += 10.0;
    }
    for (int col = 0; col < 5; ++col) {
      waypoints_->setItem(row, col, new QTableWidgetItem(numberText(values[col])));
    }
  });
  QObject::connect(remove_waypoint, &QPushButton::clicked, this, [this] {
    int row = waypoints_->currentRow();
    if (row < 0) {
      row = waypoints_->rowCount() - 1;
    }
    if (row >= 0) {
      waypoints_->removeRow(row);
      markDirty();
    }
  });

  auto* fit_key = new QShortcut(Qt::Key_F, this);
  QObject::connect(fit_key, &QShortcut::activated, this, [this] { frameView(); });
}

QDoubleSpinBox* MainWindow::addDouble(QFormLayout* form, const QString& label, double minimum,
                                      double maximum, const QString& suffix) {
  auto* box = new DoubleBox(form->parentWidget());
  box->setRange(minimum, maximum);
  box->setDecimals(8);
  box->setSingleStep(0.1);
  box->setSuffix(suffix);
  box->setFocusPolicy(Qt::StrongFocus);
  form->addRow(label, box);
  QObject::connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                   [this](double) { markDirty(); });
  return box;
}

QSpinBox* MainWindow::addInt(QFormLayout* form, const QString& label, int minimum, int maximum,
                             const QString& suffix) {
  auto* box = new IntBox(form->parentWidget());
  box->setRange(minimum, maximum);
  box->setSuffix(suffix);
  box->setFocusPolicy(Qt::StrongFocus);
  form->addRow(label, box);
  QObject::connect(box, qOverload<int>(&QSpinBox::valueChanged), this,
                   [this](int) { markDirty(); });
  return box;
}

void MainWindow::addVector(QFormLayout* form, const QString& label, QDoubleSpinBox*& x,
                           QDoubleSpinBox*& y, QDoubleSpinBox*& z, double minimum, double maximum,
                           const QString& suffix) {
  auto* row = new QWidget(form->parentWidget());
  auto* layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  const QString tips[] = {"x", "y", "z"};
  QDoubleSpinBox** boxes[] = {&x, &y, &z};
  for (int i = 0; i < 3; ++i) {
    auto* box = new DoubleBox(row);
    box->setRange(minimum, maximum);
    box->setDecimals(8);
    box->setSingleStep(0.1);
    box->setSuffix(suffix);
    box->setFocusPolicy(Qt::StrongFocus);
    box->setToolTip(tips[i]);
    QObject::connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
                     [this](double) { markDirty(); });
    layout->addWidget(box);
    *boxes[i] = box;
  }
  form->addRow(label, row);
}

void MainWindow::markDirty() {
  if (filling_form_) {
    return;
  }
  dirty_ = true;
  if (apply_button_ != nullptr) {
    apply_button_->setText("Apply*");
  }
}

MotionMode MainWindow::currentMode() const {
  return static_cast<MotionMode>(motion_mode_->currentData().toInt());
}

void MainWindow::setWaypoints(const std::vector<WaypointSpec>& waypoints) {
  QSignalBlocker blocker(waypoints_);
  waypoints_->setRowCount(static_cast<int>(waypoints.size()));
  for (int row = 0; row < waypoints_->rowCount(); ++row) {
    const WaypointSpec& waypoint = waypoints[static_cast<std::size_t>(row)];
    const double values[] = {waypoint.time, waypoint.position.x(), waypoint.position.y(),
                             waypoint.position.z(), waypoint.yaw_deg};
    for (int col = 0; col < 5; ++col) {
      waypoints_->setItem(row, col, new QTableWidgetItem(numberText(values[col])));
    }
  }
}

void MainWindow::fillForm(const SimulationConfig& config) {
  filling_form_ = true;
  const auto set = [](QDoubleSpinBox* box, double value) { box->setValue(value); };
  const auto set3 = [&](QDoubleSpinBox* x, QDoubleSpinBox* y, QDoubleSpinBox* z,
                        const Eigen::Vector3d& value) {
    x->setValue(value.x());
    y->setValue(value.y());
    z->setValue(value.z());
  };

  set(scan_rate_, config.lidar.scan_rate);
  azimuth_steps_->setValue(config.lidar.num_azimuth_steps);
  num_rings_->setValue(config.lidar.num_rings);
  set(min_elevation_, config.lidar.min_elevation_deg);
  set(max_elevation_, config.lidar.max_elevation_deg);
  set(min_range_, config.lidar.min_range);
  set(max_range_, config.lidar.max_range);
  set(intensity_, config.lidar.intensity);
  num_threads_->setValue(config.lidar.num_threads);
  set3(lidar_x_, lidar_y_, lidar_z_, config.lidar.initial_position);
  set3(lidar_roll_, lidar_pitch_, lidar_yaw_, config.lidar.initial_rpy_deg);

  set(motion_rate_, config.motion.update_rate);
  set(motion_gravity_, config.motion.gravity);
  set(start_delay_, config.motion.start_delay);
  set3(motion_x_, motion_y_, motion_z_, config.motion.initial_position);
  set3(motion_roll_, motion_pitch_, motion_yaw_, config.motion.initial_rpy_deg);
  set3(motion_mount_roll_, motion_mount_pitch_, motion_mount_yaw_, config.motion.lidar_mount_rpy_deg);
  set3(circle_x_, circle_y_, circle_z_, config.motion.trajectory_center);
  set(circle_radius_, config.motion.trajectory_radius);
  set(circle_linear_speed_, config.motion.trajectory_linear_speed);
  set3(circle_normal_x_, circle_normal_y_, circle_normal_z_, config.motion.trajectory_normal);
  set(position_gain_, config.motion.position_gain);
  set(velocity_gain_, config.motion.velocity_gain);

  set(drone_rate_, config.drone.update_rate);
  set(drone_gravity_, config.drone.gravity);
  set(drag_, config.drone.drag_coefficient);
  set(hover_, config.drone.hover_duration);
  set(min_altitude_, config.drone.min_attitude_control_altitude);
  set3(drone_x_, drone_y_, drone_z_, config.drone.initial_position);
  set3(drone_roll_, drone_pitch_, drone_yaw_, config.drone.initial_rpy_deg);
  set3(drone_mount_roll_, drone_mount_pitch_, drone_mount_yaw_, config.drone.lidar_mount_rpy_deg);
  set(cruise_speed_, config.drone.cruise_speed);
  set(distance_gain_, config.drone.distance_gain);
  set(drone_velocity_gain_, config.drone.velocity_gain);
  set(attitude_gain_, config.drone.attitude_gain);
  set(max_thrust_, config.drone.max_thrust_to_weight);
  set(max_yaw_rate_, config.drone.max_yaw_rate);
  setWaypoints(config.drone.waypoints);

  filling_form_ = false;
  dirty_ = false;
  if (apply_button_ != nullptr) {
    apply_button_->setText("Apply");
  }
}

SimulationConfig MainWindow::readForm() const {
  SimulationConfig config = loaded_config_;
  const auto vec3 = [](const QDoubleSpinBox* x, const QDoubleSpinBox* y, const QDoubleSpinBox* z) {
    return Eigen::Vector3d(x->value(), y->value(), z->value());
  };
  config.lidar.scan_rate = scan_rate_->value();
  config.lidar.num_azimuth_steps = azimuth_steps_->value();
  config.lidar.num_rings = num_rings_->value();
  config.lidar.min_elevation_deg = min_elevation_->value();
  config.lidar.max_elevation_deg = max_elevation_->value();
  config.lidar.min_range = min_range_->value();
  config.lidar.max_range = max_range_->value();
  config.lidar.intensity = intensity_->value();
  config.lidar.num_threads = num_threads_->value();
  config.lidar.initial_position = vec3(lidar_x_, lidar_y_, lidar_z_);
  config.lidar.initial_rpy_deg = vec3(lidar_roll_, lidar_pitch_, lidar_yaw_);

  config.motion.update_rate = motion_rate_->value();
  config.motion.gravity = motion_gravity_->value();
  config.motion.start_delay = start_delay_->value();
  config.motion.initial_position = vec3(motion_x_, motion_y_, motion_z_);
  config.motion.initial_rpy_deg = vec3(motion_roll_, motion_pitch_, motion_yaw_);
  config.motion.lidar_mount_rpy_deg =
      vec3(motion_mount_roll_, motion_mount_pitch_, motion_mount_yaw_);
  config.motion.trajectory_center = vec3(circle_x_, circle_y_, circle_z_);
  config.motion.trajectory_radius = circle_radius_->value();
  config.motion.trajectory_linear_speed = circle_linear_speed_->value();
  config.motion.trajectory_normal = vec3(circle_normal_x_, circle_normal_y_, circle_normal_z_);
  config.motion.position_gain = position_gain_->value();
  config.motion.velocity_gain = velocity_gain_->value();

  config.drone.update_rate = drone_rate_->value();
  config.drone.gravity = drone_gravity_->value();
  config.drone.drag_coefficient = drag_->value();
  config.drone.hover_duration = hover_->value();
  config.drone.min_attitude_control_altitude = min_altitude_->value();
  config.drone.initial_position = vec3(drone_x_, drone_y_, drone_z_);
  config.drone.initial_rpy_deg = vec3(drone_roll_, drone_pitch_, drone_yaw_);
  config.drone.lidar_mount_rpy_deg = vec3(drone_mount_roll_, drone_mount_pitch_, drone_mount_yaw_);
  config.drone.cruise_speed = cruise_speed_->value();
  config.drone.distance_gain = distance_gain_->value();
  config.drone.velocity_gain = drone_velocity_gain_->value();
  config.drone.attitude_gain = attitude_gain_->value();
  config.drone.max_thrust_to_weight = max_thrust_->value();
  config.drone.max_yaw_rate = max_yaw_rate_->value();

  config.drone.waypoints.clear();
  for (int row = 0; row < waypoints_->rowCount(); ++row) {
    bool any = false;
    bool complete = true;
    double values[5] = {};
    for (int col = 0; col < 5; ++col) {
      const QTableWidgetItem* item = waypoints_->item(row, col);
      const QString text = item == nullptr ? QString() : item->text().trimmed();
      if (text.isEmpty()) {
        complete = false;
        continue;
      }
      any = true;
      bool ok = false;
      values[col] = text.toDouble(&ok);
      if (!ok) {
        throw std::runtime_error("Waypoint " + std::to_string(row + 1) + ": '" +
                                 text.toStdString() + "' is not a number");
      }
    }
    if (!any) {
      continue;
    }
    if (!complete) {
      throw std::runtime_error("Waypoint " + std::to_string(row + 1) + " is missing a value");
    }
    WaypointSpec waypoint;
    waypoint.time = values[0];
    waypoint.position = Eigen::Vector3d(values[1], values[2], values[3]);
    waypoint.yaw_deg = values[4];
    config.drone.waypoints.push_back(waypoint);
  }
  return config;
}

void MainWindow::validate(const SimulationConfig& config) const {
  auto check = [](const char* name, auto&& build) {
    try {
      build();
    } catch (const std::exception& error) {
      throw std::runtime_error(std::string(name) + ": " + error.what());
    }
  };
  check("LiDAR", [&] { LidarScanner scanner(config.lidarConfig()); });
  if (!(config.motion.update_rate > 0.0)) {
    throw std::runtime_error("Circle: update_rate must be positive");
  }
  if (!(config.drone.update_rate > 0.0)) {
    throw std::runtime_error("Drone: update_rate must be positive");
  }
  check("Circle", [&] { MotionSimulator simulator(config.motionConfig()); });
  check("Drone", [&] { DroneSimulator simulator(config.droneConfig()); });
}

void MainWindow::applyWithWorld(const std::shared_ptr<const World>& world) {
  if (!world) {
    throw std::runtime_error("Load a world file first");
  }
  SimulationConfig config = readForm();
  validate(config);
  const MotionMode mode = currentMode();
  const bool was_playing = session_.playing();
  session_.start(config, mode, world);
  loaded_config_ = std::move(config);
  world_ = world;
  running_mode_ = mode;
  object_count_ = world_->size();
  dirty_ = false;
  started_ = true;
  last_error_.clear();
  point_revision_ = 0;
  line_revision_ = 0;
  if (apply_button_ != nullptr) {
    apply_button_->setText("Apply");
  }
  if (was_playing && mode != MotionMode::Static) {
    session_.setPlaying(true);
  }
}

void MainWindow::loadFiles(const QString& world_path, const QString& params_path) {
  world_path_->setText(world_path);
  params_path_->setText(params_path);
  try {
    SimulationConfig config = loadSimulationConfig(params_path.toStdString());
    auto world = std::make_shared<const World>(loadWorldFile(world_path.toStdString()));
    SceneMesh mesh = buildSceneMesh(world_path.toStdString());
    fillForm(config);
    loaded_config_ = std::move(config);
    bounds_min_ = mesh.bounds_min;
    bounds_max_ = mesh.bounds_max;
    view_->setScene(std::move(mesh));
    applyWithWorld(world);
    frameView();
    setWindowTitle(QString("lidar_sim — %1").arg(QFileInfo(world_path).fileName()));
  } catch (const std::exception& error) {
    QMessageBox::critical(this, "lidar_sim", QString::fromStdString(error.what()));
  }
}

void MainWindow::reloadWorld() {
  try {
    const QString path = world_path_->text();
    auto world = std::make_shared<const World>(loadWorldFile(path.toStdString()));
    SceneMesh mesh = buildSceneMesh(path.toStdString());
    applyWithWorld(world);
    bounds_min_ = mesh.bounds_min;
    bounds_max_ = mesh.bounds_max;
    view_->setScene(std::move(mesh));
    frameView();
    setWindowTitle(QString("lidar_sim — %1").arg(QFileInfo(path).fileName()));
  } catch (const std::exception& error) {
    showError(this, error);
  }
}

void MainWindow::loadParams() {
  try {
    SimulationConfig config = loadSimulationConfig(params_path_->text().toStdString());
    fillForm(config);
    loaded_config_ = config;
    if (world_) {
      applyWithWorld(world_);
    }
    statusBar()->showMessage("Loaded " + params_path_->text());
  } catch (const std::exception& error) {
    showError(this, error);
  }
}

void MainWindow::saveParams() {
  try {
    if (params_path_->text().isEmpty()) {
      const QString path = QFileDialog::getSaveFileName(this, "Save parameters", "params.yaml",
                                                       "YAML (*.yaml *.yml)");
      if (path.isEmpty()) {
        return;
      }
      params_path_->setText(path);
    }
    SimulationConfig config = readForm();
    validate(config);
    saveSimulationConfig(params_path_->text().toStdString(), config);
    loaded_config_ = config;
    if (world_) {
      applyWithWorld(world_);
    }
    statusBar()->showMessage("Saved " + params_path_->text());
  } catch (const std::exception& error) {
    showError(this, error);
  }
}

void MainWindow::browseWorld() {
  const QString path = QFileDialog::getOpenFileName(this, "World file", world_path_->text(),
                                                   "YAML (*.yaml *.yml)");
  if (path.isEmpty()) {
    return;
  }
  world_path_->setText(path);
  reloadWorld();
}

void MainWindow::browseParams() {
  const QString path = QFileDialog::getOpenFileName(this, "Parameters", params_path_->text(),
                                                   "YAML (*.yaml *.yml)");
  if (path.isEmpty()) {
    return;
  }
  params_path_->setText(path);
  loadParams();
}

void MainWindow::play() {
  try {
    if (dirty_) {
      applyWithWorld(world_);
    }
  } catch (const std::exception& error) {
    showError(this, error);
    return;
  }
  if (!started_) {
    statusBar()->showMessage("Load a world file and a parameter file first.");
    return;
  }
  if (running_mode_ == MotionMode::Static) {
    statusBar()->showMessage(
        "Static pose does not move. Choose Circle or Drone, then Apply or Play.");
    return;
  }
  session_.setPlaying(true);
}

void MainWindow::pause() { session_.setPlaying(false); }

void MainWindow::frameView() {
  Eigen::Vector3d low = bounds_min_;
  Eigen::Vector3d high = bounds_max_;
  const auto include = [&](const Eigen::Vector3d& point) {
    low = low.cwiseMin(point);
    high = high.cwiseMax(point);
  };
  try {
    const SimulationConfig config = readForm();
    if (currentMode() == MotionMode::Drone) {
      include(config.drone.initial_position);
      for (const WaypointSpec& waypoint : config.drone.waypoints) {
        include(waypoint.position);
      }
    } else if (currentMode() == MotionMode::Circle) {
      include(config.motion.initial_position);
      include(config.motion.trajectory_center);
      const double radius = std::abs(config.motion.trajectory_radius);
      include(config.motion.trajectory_center + Eigen::Vector3d::Constant(radius));
      include(config.motion.trajectory_center - Eigen::Vector3d::Constant(radius));
    } else {
      include(config.lidar.initial_position);
    }
  } catch (const std::exception&) {
  }
  view_->frameScene(low, high);
}

void MainWindow::onTick() {
  if (!started_) {
    return;
  }
  SimulationSession::Snapshot snap = session_.snapshotFor(point_revision_, line_revision_);
  if (snap.point_revision != point_revision_) {
    point_revision_ = snap.point_revision;
    view_->setPoints(std::move(snap.points));
  }
  if (snap.line_revision != line_revision_) {
    line_revision_ = snap.line_revision;
    view_->setSensorLines(std::move(snap.sensor_lines));
    view_->setPathLines(std::move(snap.path_lines));
  }
  if (snap.error != last_error_) {
    last_error_ = snap.error;
    if (!last_error_.empty()) {
      session_.setPlaying(false);
      QMessageBox::warning(this, "lidar_sim", QString::fromStdString(last_error_));
    }
  }
  if (!shader_reported_ && !view_->shaderError().isEmpty()) {
    shader_reported_ = true;
    QMessageBox::critical(this, "lidar_sim",
                          "The 3D view could not start:\n" + view_->shaderError());
  }

  const char* mode = "static";
  if (running_mode_ == MotionMode::Circle) {
    mode = "circle";
  } else if (running_mode_ == MotionMode::Drone) {
    mode = "drone";
  }
  const QString state = session_.playing() ? "Playing" : "Paused";
  QString text = QString("%1 | %2 | t = %3 s | %4 points | scan %5 ms | %6 objects")
                     .arg(state)
                     .arg(mode)
                     .arg(snap.sim_time, 0, 'f', 2)
                     .arg(snap.valid_points)
                     .arg(snap.scan_ms, 0, 'f', 0)
                     .arg(static_cast<qulonglong>(object_count_));
  const double period_ms = 1000.0 / std::max(loaded_config_.lidar.scan_rate, 1e-6);
  if (snap.scan_ms > period_ms && snap.valid_points > 0) {
    text += QString(" | slower than the %1 ms period; lower azimuth steps for a faster view")
                .arg(period_ms, 0, 'f', 0);
  }
  play_button_->setEnabled(!session_.playing());
  pause_button_->setEnabled(session_.playing());
  statusBar()->showMessage(text);
}

std::uint64_t MainWindow::scanRevision() const { return session_.pointRevision(); }

int MainWindow::validPoints() const { return session_.validPoints(); }

QImage MainWindow::grabView() {
  view_->repaint();
  return view_->grabFramebuffer();
}

}  // namespace lidar_sim
