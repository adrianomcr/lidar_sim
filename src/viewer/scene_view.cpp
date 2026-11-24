#include "scene_view.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

namespace lidar_sim {
namespace {

const char* kVertexShader = R"(#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;
uniform mat4 mvp;
uniform float point_size;
uniform float depth_bias;
out vec3 v_normal;
out vec3 v_color;
void main() {
  gl_Position = mvp * vec4(position, 1.0);
  gl_Position.z -= depth_bias * gl_Position.w;
  v_normal = normal;
  v_color = color;
  gl_PointSize = point_size;
}
)";

const char* kFragmentShader = R"(#version 330 core
in vec3 v_normal;
in vec3 v_color;
uniform int lighting;
uniform float alpha;
out vec4 frag_color;
void main() {
  vec3 color = v_color;
  if (lighting == 1) {
    vec3 light = normalize(vec3(0.30, 0.45, 0.84));
    float shade = 0.38 + 0.62 * abs(dot(normalize(v_normal), light));
    color *= shade;
  }
  frag_color = vec4(color, alpha);
}
)";

QPoint eventPosition(QMouseEvent* event) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
  return event->position().toPoint();
#else
  return event->pos();
#endif
}

Eigen::Matrix4f perspective(float fov_y_deg, float aspect, float near_z, float far_z) {
  const float f = 1.0f / std::tan(fov_y_deg * static_cast<float>(M_PI) / 360.0f);
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Zero();
  matrix(0, 0) = f / aspect;
  matrix(1, 1) = f;
  matrix(2, 2) = (far_z + near_z) / (near_z - far_z);
  matrix(3, 2) = -1.0f;
  matrix(2, 3) = (2.0f * far_z * near_z) / (near_z - far_z);
  return matrix;
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f& eye, const Eigen::Vector3f& center,
                       const Eigen::Vector3f& up) {
  const Eigen::Vector3f forward = (center - eye).normalized();
  const Eigen::Vector3f side = forward.cross(up).normalized();
  const Eigen::Vector3f camera_up = side.cross(forward);
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix(0, 0) = side.x();
  matrix(0, 1) = side.y();
  matrix(0, 2) = side.z();
  matrix(0, 3) = -side.dot(eye);
  matrix(1, 0) = camera_up.x();
  matrix(1, 1) = camera_up.y();
  matrix(1, 2) = camera_up.z();
  matrix(1, 3) = -camera_up.dot(eye);
  matrix(2, 0) = -forward.x();
  matrix(2, 1) = -forward.y();
  matrix(2, 2) = -forward.z();
  matrix(2, 3) = forward.dot(eye);
  return matrix;
}

}  // namespace

SceneView::SceneView(QWidget* parent) : QOpenGLWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMinimumSize(320, 240);
  setToolTip("Left drag orbits, right drag pans, wheel zooms. F frames the scene, Space plays.");
}

SceneView::~SceneView() {
  if (!functions_ready_ || context() == nullptr) {
    return;
  }
  makeCurrent();
  for (Buffer* buffer : {&opaque_, &planes_, &points_, &sensor_, &path_}) {
    glDeleteBuffers(1, &buffer->vbo);
    if (buffer->indexed) {
      glDeleteBuffers(1, &buffer->ebo);
    }
    glDeleteVertexArrays(1, &buffer->vao);
  }
  doneCurrent();
}

void SceneView::setScene(SceneMesh mesh) {
  opaque_vertices_ = std::move(mesh.opaque_vertices);
  opaque_indices_ = std::move(mesh.opaque_indices);
  plane_vertices_ = std::move(mesh.plane_vertices);
  plane_indices_ = std::move(mesh.plane_indices);
  opaque_dirty_ = true;
  planes_dirty_ = true;
  frameScene(mesh.bounds_min, mesh.bounds_max);
}

void SceneView::setPoints(std::vector<float> points) {
  point_vertices_ = std::move(points);
  points_dirty_ = true;
  update();
}

void SceneView::setSensorLines(std::vector<float> lines) {
  sensor_lines_ = std::move(lines);
  sensor_dirty_ = true;
  update();
}

void SceneView::setPathLines(std::vector<float> lines) {
  path_lines_ = std::move(lines);
  path_dirty_ = true;
  update();
}

void SceneView::frameScene(const Eigen::Vector3d& min_corner, const Eigen::Vector3d& max_corner) {
  framed_min_ = min_corner;
  framed_max_ = max_corner;
  has_frame_ = true;
  const Eigen::Vector3d center = 0.5 * (min_corner + max_corner);
  target_ = center.cast<float>();
  scene_radius_ = std::max(1.0f, static_cast<float>(0.5 * (max_corner - min_corner).norm()));
  distance_ = scene_radius_ * 1.65f;
  yaw_deg_ = 38.0f;
  pitch_deg_ = 26.0f;
  update();
}

void SceneView::setShowWorld(bool enabled) {
  show_world_ = enabled;
  update();
}

void SceneView::setShowPoints(bool enabled) {
  show_points_ = enabled;
  update();
}

void SceneView::setShowSensor(bool enabled) {
  show_sensor_ = enabled;
  update();
}

void SceneView::setShowPath(bool enabled) {
  show_path_ = enabled;
  update();
}

void SceneView::setWireframe(bool enabled) {
  wireframe_ = enabled;
  update();
}

void SceneView::setPointSize(float pixels) {
  point_size_ = std::max(1.0f, pixels);
  update();
}

void SceneView::initializeGL() {
  initializeOpenGLFunctions();
  functions_ready_ = true;
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);
  polygon_mode_ =
      reinterpret_cast<void (*)(GLenum, GLenum)>(context()->getProcAddress("glPolygonMode"));

  if (!program_.addShaderFromSourceCode(QOpenGLShader::Vertex, kVertexShader) ||
      !program_.addShaderFromSourceCode(QOpenGLShader::Fragment, kFragmentShader) ||
      !program_.link()) {
    shader_error_ = program_.log();
    return;
  }
  mvp_location_ = program_.uniformLocation("mvp");
  point_size_location_ = program_.uniformLocation("point_size");
  depth_bias_location_ = program_.uniformLocation("depth_bias");
  lighting_location_ = program_.uniformLocation("lighting");
  alpha_location_ = program_.uniformLocation("alpha");

  createBuffer(opaque_, true);
  createBuffer(planes_, true);
  createBuffer(points_, false);
  createBuffer(sensor_, false);
  createBuffer(path_, false);
  gl_ready_ = true;
}

void SceneView::resizeGL(int width, int height) {
  viewport_width_ = std::max(width, 1);
  viewport_height_ = std::max(height, 1);
}

void SceneView::createBuffer(Buffer& buffer, bool indexed) {
  buffer.indexed = indexed;
  glGenVertexArrays(1, &buffer.vao);
  glGenBuffers(1, &buffer.vbo);
  glBindVertexArray(buffer.vao);
  glBindBuffer(GL_ARRAY_BUFFER, buffer.vbo);
  const int stride = 9 * static_cast<int>(sizeof(float));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
                        reinterpret_cast<void*>(6 * sizeof(float)));
  if (indexed) {
    glGenBuffers(1, &buffer.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.ebo);
  }
  glBindVertexArray(0);
}

void SceneView::upload(Buffer& buffer, const std::vector<float>& vertices,
                       const std::vector<unsigned int>* indices) {
  glBindBuffer(GL_ARRAY_BUFFER, buffer.vbo);
  glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
               vertices.empty() ? nullptr : vertices.data(), GL_DYNAMIC_DRAW);
  if (indices != nullptr) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices->size() * sizeof(unsigned int)),
                 indices->empty() ? nullptr : indices->data(), GL_DYNAMIC_DRAW);
    buffer.count = static_cast<int>(indices->size());
  } else {
    buffer.count = static_cast<int>(vertices.size() / 9);
  }
}

void SceneView::draw(const Buffer& buffer, GLenum mode) {
  if (buffer.count <= 0) {
    return;
  }
  glBindVertexArray(buffer.vao);
  if (buffer.indexed) {
    glDrawElements(mode, buffer.count, GL_UNSIGNED_INT, nullptr);
  } else {
    glDrawArrays(mode, 0, buffer.count);
  }
}

Eigen::Matrix4f SceneView::viewMatrix() const {
  const float yaw = yaw_deg_ * static_cast<float>(M_PI) / 180.0f;
  const float pitch = pitch_deg_ * static_cast<float>(M_PI) / 180.0f;
  Eigen::Vector3f offset(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
                         std::sin(pitch));
  return lookAt(target_ + offset * distance_, target_, Eigen::Vector3f::UnitZ());
}

Eigen::Vector3f SceneView::cameraRight() const {
  const float yaw = yaw_deg_ * static_cast<float>(M_PI) / 180.0f;
  const float pitch = pitch_deg_ * static_cast<float>(M_PI) / 180.0f;
  const Eigen::Vector3f forward(-std::cos(pitch) * std::cos(yaw), -std::cos(pitch) * std::sin(yaw),
                                -std::sin(pitch));
  Eigen::Vector3f right = forward.cross(Eigen::Vector3f::UnitZ());
  if (right.norm() < 1e-4f) {
    return Eigen::Vector3f::UnitX();
  }
  return right.normalized();
}

Eigen::Vector3f SceneView::cameraUp() const {
  const float yaw = yaw_deg_ * static_cast<float>(M_PI) / 180.0f;
  const float pitch = pitch_deg_ * static_cast<float>(M_PI) / 180.0f;
  const Eigen::Vector3f forward(-std::cos(pitch) * std::cos(yaw), -std::cos(pitch) * std::sin(yaw),
                                -std::sin(pitch));
  return cameraRight().cross(forward).normalized();
}

void SceneView::paintGL() {
  if (!gl_ready_ || !program_.isLinked()) {
    glClearColor(0.12f, 0.13f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    return;
  }
  if (opaque_dirty_) {
    upload(opaque_, opaque_vertices_, &opaque_indices_);
    opaque_dirty_ = false;
  }
  if (planes_dirty_) {
    upload(planes_, plane_vertices_, &plane_indices_);
    planes_dirty_ = false;
  }
  if (points_dirty_) {
    upload(points_, point_vertices_, nullptr);
    points_dirty_ = false;
  }
  if (sensor_dirty_) {
    upload(sensor_, sensor_lines_, nullptr);
    sensor_dirty_ = false;
  }
  if (path_dirty_) {
    upload(path_, path_lines_, nullptr);
    path_dirty_ = false;
  }

  glClearColor(0.12f, 0.13f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glDisable(GL_BLEND);

  const float aspect =
      static_cast<float>(viewport_width_) / static_cast<float>(std::max(viewport_height_, 1));
  const float near_z = std::max(0.05f, distance_ * 0.0015f);
  const float far_z = distance_ + scene_radius_ * 8.0f + 20.0f;
  const Eigen::Matrix4f mvp = perspective(45.0f, aspect, near_z, far_z) * viewMatrix();
  const float pixel_scale = static_cast<float>(devicePixelRatioF());

  program_.bind();
  glUniformMatrix4fv(mvp_location_, 1, GL_FALSE, mvp.data());
  glUniform1f(point_size_location_, point_size_ * pixel_scale);
  glUniform1f(depth_bias_location_, 0.0f);
  glUniform1i(lighting_location_, 1);
  glUniform1f(alpha_location_, 1.0f);

  if (show_world_) {
    if (polygon_mode_ != nullptr) {
      polygon_mode_(GL_FRONT_AND_BACK, wireframe_ ? GL_LINE : GL_FILL);
    }
    draw(opaque_, GL_TRIANGLES);
    if (polygon_mode_ != nullptr) {
      polygon_mode_(GL_FRONT_AND_BACK, GL_FILL);
    }
  }

  if (show_points_) {
    glUniform1i(lighting_location_, 0);
    glUniform1f(depth_bias_location_, 0.0012f);
    draw(points_, GL_POINTS);
    glUniform1f(depth_bias_location_, 0.0f);
  }

  if (show_world_) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glUniform1i(lighting_location_, 1);
    glUniform1f(alpha_location_, 0.38f);
    draw(planes_, GL_TRIANGLES);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glUniform1f(alpha_location_, 1.0f);
  }

  glUniform1i(lighting_location_, 0);
  if (show_path_) {
    draw(path_, GL_LINES);
  }
  if (show_sensor_) {
    draw(sensor_, GL_LINES);
  }
  program_.release();
}

void SceneView::mousePressEvent(QMouseEvent* event) {
  last_pos_ = eventPosition(event);
  setFocus();
  event->accept();
}

void SceneView::mouseMoveEvent(QMouseEvent* event) {
  const QPoint pos = eventPosition(event);
  const QPoint delta = pos - last_pos_;
  last_pos_ = pos;
  if (event->buttons() & Qt::LeftButton) {
    yaw_deg_ -= static_cast<float>(delta.x()) * 0.35f;
    pitch_deg_ = std::clamp(pitch_deg_ + static_cast<float>(delta.y()) * 0.30f, -85.0f, 85.0f);
    update();
  } else if (event->buttons() & (Qt::RightButton | Qt::MiddleButton)) {
    const float scale = distance_ * 0.0018f;
    target_ -= cameraRight() * (static_cast<float>(delta.x()) * scale);
    target_ += cameraUp() * (static_cast<float>(delta.y()) * scale);
    update();
  }
}

void SceneView::wheelEvent(QWheelEvent* event) {
  const float steps = static_cast<float>(event->angleDelta().y()) / 120.0f;
  distance_ = std::clamp(distance_ * std::pow(0.86f, steps), 0.3f, 5000.0f);
  update();
  event->accept();
}

void SceneView::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_F && has_frame_) {
    frameScene(framed_min_, framed_max_);
    return;
  }
  if (event->key() == Qt::Key_Space && on_space_) {
    on_space_();
    return;
  }
  QOpenGLWidget::keyPressEvent(event);
}

}  // namespace lidar_sim
