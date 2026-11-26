# Conventions

## Units

SI everywhere: meters, seconds, radians, m/s, rad/s and m/s². The exceptions are explicit:

- Parameters in `config/params.yaml` whose name ends in `_deg` are in degrees and are converted
  when they are read.
- `angles` in the world file are in radians.

## Frames

| Frame | Name in ROS | Definition |
| --- | --- | --- |
| World | `world` (`world_frame`) | Fixed frame, z up. Gravity is $-g\thinspace\hat z$. |
| LiDAR | `lidar` (`lidar_frame`) | Sensor frame: x at azimuth 0, y to the left, z along the spin axis. The IMU and the point cloud use this frame. |
| LiDAR ground truth | `tf_child_frame`, default `lidar` | Child of the `world` TF published by `sim_lidar`. Set it to `lidar_gt` when another node also parents `lidar`. |
| Drone | `drone` (`drone_frame`) | Quadrotor body: x forward, z along the thrust. |
| Primitive | none | Local frame of each object in the world file, where its canonical shape is defined. |

The LiDAR is mounted on the moving body with a fixed rotation $q_m$ (`lidar_mount_rpy_deg`, LiDAR
to body). Its position is the body origin; there is no lever arm.

## Rotations from roll, pitch and yaw

Roll $\phi$, pitch $\theta$ and yaw $\psi$ (the world file's `angles` and every `*_rpy_deg`
parameter) build the rotation

```math
R(\phi, \theta, \psi) = R_z(\psi)\, R_y(\theta)\, R_x(\phi)
```

```math
R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}
\quad
R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}
\quad
R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}
```

This is the usual aerospace Z-Y-X convention: yaw, then pitch, then roll about the moving axes
(equivalently roll, pitch, yaw about the fixed axes). It is implemented by
`quaternionFromRpy()` in `math_utils.h`.

$R$ rotates vectors from the local frame to the parent frame. A frame with origin $c$ and
orientation $R$ maps points as

```math
p_{\text{parent}} = c + R\, p_{\text{local}}, \qquad p_{\text{local}} = R^\top (p_{\text{parent}} - c).
```

Useful special cases for objects in the world file:

| `angles` | Local z axis in the world | Typical use |
| --- | --- | --- |
| `[0, 0, 0]` | $+z$ | Floor, vertical columns |
| `[0, pi/2, 0]` | $+x$ | Walls facing x, beams along x |
| `[pi/2, 0, 0]` | $-y$ | Walls facing y, beams along y |

## Quaternions

Orientations are unit quaternions $q = (w, x, y, z)$ with the Hamilton product, the convention of
Eigen and ROS. A quaternion rotates from the body frame to the world frame:

```math
v_{\text{world}} = q \otimes (0, v_{\text{body}}) \otimes q^{*}.
```

Composition reads right to left: the LiDAR orientation is
$q_{\text{lidar}} = q_{\text{body}} \otimes q_m$.

Watch out for the storage order in Eigen: the constructor is `Quaterniond(w, x, y, z)` but
`coeffs()` returns `(x, y, z, w)`.

## Angular velocity and acceleration

- Angular velocities are expressed in the body frame (what a gyroscope measures).
- Linear velocities and accelerations of the bodies are expressed in the world frame.
- The accelerometer measures **specific force**, the non-gravitational acceleration
  $f = a - g_{\text{vec}} = a + g\thinspace\hat z$, expressed in the sensor frame. A sensor at rest
  reads $+g$ along its up axis.
