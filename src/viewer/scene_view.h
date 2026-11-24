#ifndef LIDAR_SIM_VIEWER_SCENE_VIEW_H
#define LIDAR_SIM_VIEWER_SCENE_VIEW_H

#include <functional>
#include <vector>

#include <Eigen/Core>

#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <QString>

#include "scene_mesh.h"

namespace lidar_sim {

/// Orbiting view of the world mesh, the point cloud and the sensor path.
class SceneView : public QOpenGLWidget, protected QOpenGLExtraFunctions {
 public:
  explicit SceneView(QWidget* parent = nullptr);
  ~SceneView() override;

  void setScene(SceneMesh mesh);
  void setPoints(std::vector<float> points);
  void setSensorLines(std::vector<float> lines);
  void setPathLines(std::vector<float> lines);
  void frameScene(const Eigen::Vector3d& min_corner, const Eigen::Vector3d& max_corner);

  void setShowWorld(bool enabled);
  void setShowPoints(bool enabled);
  void setShowSensor(bool enabled);
  void setShowPath(bool enabled);
  void setWireframe(bool enabled);
  void setPointSize(float pixels);

  /// Called for Space when this widget has focus. F always frames the last scene.
  void setSpaceHandler(std::function<void()> handler) { on_space_ = std::move(handler); }

  const QString& shaderError() const { return shader_error_; }

 protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;

 private:
  struct Buffer {
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint ebo = 0;
    int count = 0;
    bool indexed = false;
  };

  void createBuffer(Buffer& buffer, bool indexed);
  void upload(Buffer& buffer, const std::vector<float>& vertices,
              const std::vector<unsigned int>* indices);
  void draw(const Buffer& buffer, GLenum mode);
  Eigen::Matrix4f viewMatrix() const;
  Eigen::Vector3f cameraRight() const;
  Eigen::Vector3f cameraUp() const;

  QOpenGLShaderProgram program_;
  QString shader_error_;
  GLint mvp_location_ = -1;
  GLint point_size_location_ = -1;
  GLint depth_bias_location_ = -1;
  GLint lighting_location_ = -1;
  GLint alpha_location_ = -1;
  void (*polygon_mode_)(GLenum, GLenum) = nullptr;

  Buffer opaque_;
  Buffer planes_;
  Buffer points_;
  Buffer sensor_;
  Buffer path_;

  std::vector<float> opaque_vertices_;
  std::vector<unsigned int> opaque_indices_;
  std::vector<float> plane_vertices_;
  std::vector<unsigned int> plane_indices_;
  std::vector<float> point_vertices_;
  std::vector<float> sensor_lines_;
  std::vector<float> path_lines_;
  bool functions_ready_ = false;
  bool opaque_dirty_ = false;
  bool planes_dirty_ = false;
  bool points_dirty_ = false;
  bool sensor_dirty_ = false;
  bool path_dirty_ = false;
  bool gl_ready_ = false;

  bool show_world_ = true;
  bool show_points_ = true;
  bool show_sensor_ = true;
  bool show_path_ = true;
  bool wireframe_ = false;
  float point_size_ = 3.0f;

  Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
  float yaw_deg_ = 40.0f;
  float pitch_deg_ = 24.0f;
  float distance_ = 40.0f;
  float scene_radius_ = 20.0f;
  int viewport_width_ = 1;
  int viewport_height_ = 1;
  QPoint last_pos_;
  Eigen::Vector3d framed_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d framed_max_ = Eigen::Vector3d::Zero();
  bool has_frame_ = false;
  std::function<void()> on_space_;
};

}  // namespace lidar_sim

#endif  // LIDAR_SIM_VIEWER_SCENE_VIEW_H
