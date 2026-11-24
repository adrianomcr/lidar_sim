# Motion models

The motion models move the LiDAR through the scene and produce what an onboard IMU and a ground
truth system would measure. Both are ROS-free classes in `include/lidar_sim/motion/` with the same
interface: `step(t, dt)` advances the simulation, `state()` returns the body state and `imu()` the
IMU reading.

| Model | Class | Node | Motion |
| --- | --- | --- | --- |
| Circle | `MotionSimulator` | `sim_motion` | Kinematic body tracking a circle, constant orientation |
| Quadrotor | `DroneSimulator` | `sim_drone` | Quadrotor with thrust and body-rate control flying waypoints |

Notation: position $x$ and velocity $v$ in the world frame, orientation $q$ from body to world
with rotation matrix $R(q)$, body angular velocity $\omega$, gravity $g$, $\hat z = (0, 0, 1)$.

## Rigid body integration

`RigidBodyState::integrate(a, dt)` advances the state with one explicit Euler step, holding the
world acceleration $a$ and the body rates $\omega$ constant over the step:

```math
\begin{aligned}
x_{k+1} &= x_k + v_k\, \Delta t
\\ v_{k+1} &= v_k + a_k\, \Delta t
\\ q_{k+1} &= \mathrm{normalize}\!\left(q_k + \tfrac{\Delta t}{2}\, q_k \otimes (0, \omega_k)\right)
\end{aligned}
```

The last line is the quaternion kinematics $\dot q = \frac{1}{2}\thinspace q \otimes (0, \omega)$ for
body-frame rates, followed by a renormalization that keeps $q$ a unit quaternion. The position
uses the velocity from before the step. These are first-order schemes, accurate enough at the
default 50 Hz for the smooth motions simulated here.

## IMU model

`measureImu(body, f, q_m)` models an ideal IMU (no noise, no bias) at the body origin, mounted with
the rotation $q_m$ (IMU to body). Given the specific force $f$ in the world frame
([Conventions](conventions.md#angular-velocity-and-acceleration)):

```math
q_{\text{imu}} = q \otimes q_m, \qquad
\omega_{\text{imu}} = R(q_m)^\top \omega, \qquad
f_{\text{imu}} = R(q_{\text{imu}})^\top f.
```

The LiDAR and the IMU share the mount, so $q_{\text{imu}}$ is also the LiDAR orientation
published on `lidar_odom_gt`. The simulators measure the IMU after integrating, so the reading
combines the new orientation with the rates and forces applied during the step.

## Circle: `MotionSimulator`

### Reference trajectory

`CircularTrajectory` is a circle of radius $\rho$ around $c$. The parameter file gives the
tangential speed $v$ and one normal $n$ of the plane. The angular speed is $\Omega = v / \rho$
when $\rho \neq 0$, and zero when the radius is zero. Positive $v$ travels by the right-hand rule
about $n$. The simulator builds two orthogonal unit axes in that plane so that
$u_1 \times u_2$ points along $n$. For $n = (0, 0, 1)$ those axes are $\hat x$ and $\hat y$,
and the circle is parallel to the $xy$ plane:

```math
p_r(t) = c + \rho\left(\cos(\Omega t)\, u_1 + \sin(\Omega t)\, u_2\right), \qquad
v_r(t) = \rho\,\Omega\left(-\sin(\Omega t)\, u_1 + \cos(\Omega t)\, u_2\right).
```

The axes are normalized when the trajectory is built, and it throws if the normal is zero or the
axes are not orthogonal. With the defaults ($c = (-12, 0, 7)$, $\rho = 6$, $v = 2.4$ m/s,
$n = \hat x$) the circle is vertical, in the plane $x = -12$. Then $\Omega = 0.4$ rad/s, the
period is 15.7 s and the speed is 2.4 m/s.

### Tracking law

The body is a double integrator commanded in acceleration. A velocity command corrects the
reference velocity with the position error, and the acceleration drives the velocity towards it:

```math
v_c = v_r(t) + k_p\,\big(p_r(t) - x\big), \qquad a = k_v\,(v_c - v).
```

Before `start_delay` ($t < t_0$) the acceleration is zero and the body waits at its initial
position. The angular velocity is always zero, so the orientation stays at `initial_rpy_deg`.
The IMU feels $f = a + g\thinspace\hat z$.

### Closed-loop behavior

Taking Laplace transforms of $\ddot x = k_v\thinspace(\dot p_r + k_p\thinspace(p_r - x) - \dot x)$:

```math
\frac{X(s)}{P_r(s)} = G(s) = \frac{k_v\,(s + k_p)}{s^2 + k_v\, s + k_v k_p}.
```

With the default gains $k_p = 0.2$, $k_v = 0.4$ the poles are $s = -0.2 \pm 0.2j$ (damping 0.71,
time constant 5 s). The reference acceleration is not fed forward, so the body does not sit on
the reference point. At the circle frequency, $|G(j\Omega)| = 1$ (because
$\Omega^2 = 2 k_v k_p$ for these values) and $\arg G(j\Omega) = -53°$: the body converges to the
**same circle**, about 53° (2.3 s) behind the reference, which stays 5.4 m ahead of it. Other gains
change both the radius and the lag; use $G(j\Omega)$ to predict them.

## Quadrotor: `DroneSimulator`

### Dynamics

The drone is a point mass with an attitude. The inputs are the mass-normalized thrust $T$ along
the body z axis and the body rates $\omega$, which are tracked perfectly (no rotational
dynamics). Linear drag with coefficient $c_d$ opposes the velocity:

```math
\dot x = v, \qquad
\dot v = R(q)\begin{bmatrix}0\\0\\T\end{bmatrix} - g\,\hat z - c_d\, v, \qquad
\dot q = \tfrac{1}{2}\, q \otimes (0, \omega).
```

These are integrated with the Euler scheme above.

### Controller

`QuadrotorController::compute(state, target)` turns the target waypoint (position $p_w$, yaw
$\psi_w$) into $T$ and $\omega$ in four stages.

**1. Velocity field.** With $e = x - p_w$ and $\delta = \lVert e \rVert$, the desired velocity
points at the waypoint with a speed that saturates far away and fades out close to it:

```math
s(\delta) = v_{\text{cruise}}\, \frac{2}{\pi} \arctan(k_d\, \delta), \qquad
v_r = -s(\delta)\, \frac{e}{\delta + \varepsilon}, \quad \varepsilon = 10^{-6}.
```

Far away $s \to v_{\text{cruise}}$; close to the waypoint
$s \approx \frac{2}{\pi} v_{\text{cruise}} k_d\thinspace\delta$, a linear law that brings the drone in
exponentially with the rate $\frac{2}{\pi} v_{\text{cruise}} k_d$ (1.06 s⁻¹ with the defaults).
$\varepsilon$ avoids a division by zero at the waypoint.

**2. Desired thrust vector.** A proportional velocity loop plus gravity compensation:

```math
f_d = k_v\,(v_r - v) + g\,\hat z.
```

**3. Reference attitude.** The body z axis must point along $f_d$, and the body x axis as close as
possible to the waypoint heading $h = (\cos\psi_w, \sin\psi_w, 0)$ (Gram-Schmidt):

```math
z_r = \frac{f_d}{\lVert f_d \rVert}, \qquad
x_r = \frac{h - (h \cdot z_r)\, z_r}{\lVert h - (h \cdot z_r)\, z_r \rVert}, \qquad
y_r = z_r \times x_r, \qquad
R_r = [\,x_r \;\; y_r \;\; z_r\,].
```

The thrust is the projection of $f_d$ on $z_r$, which is simply its norm, limited to $\kappa$
times gravity (`max_thrust_to_weight`):

```math
T = \min\left(f_d \cdot z_r,\; \kappa\, g\right).
```

The thrust is applied along the **current** body z axis, not along $z_r$: horizontal acceleration
only builds up as the attitude loop tilts the drone.

**4. Attitude loop.** The attitude error, expressed in the body frame, is
$q_e = q^{\ast} \otimes q_r$, with axis $u$ and angle $\theta \in [0, \pi]$. The body rates rotate
the drone towards the reference:

```math
\omega = k_a\, \sin\theta\; u, \qquad
\omega_z \leftarrow \mathrm{clamp}(\omega_z, -\omega_{z,\max}, \omega_{z,\max}).
```

For small errors $\omega \approx k_a\thinspace\theta\thinspace u$, a proportional loop with a time constant of
$1/k_a$ (0.33 s with the defaults). Writing $q_e = (\cos\frac{\theta}{2}, \sin\frac{\theta}{2} u)$
shows that $\sin\theta\thinspace u = 2\thinspace q_{e,w}\thinspace q_{e,\text{vec}}$, the classic quaternion-error
feedback. The yaw rate limit makes heading changes slow and smooth.

The construction of $x_r$ breaks down if $f_d$ is horizontal, which would require the drone to
climb about $g / k_v$ faster than commanded; it does not happen in normal operation.

### Flight phases

`DroneSimulator::step(t, dt)` overrides the controller during take-off:

| Condition | Command | Effect |
| --- | --- | --- |
| $t <$ `hover_duration` | $T = g$, $\omega = 0$ | Hovers at the initial position (it starts at rest) |
| $z <$ `min_attitude_control_altitude` | $\omega = 0$ | Attitude frozen (level), so the drone climbs straight up |
| otherwise | controller output | Waypoint tracking |

The IMU uses the specific force of the thrust, $f = R(q)\thinspace(0, 0, T)$. Drag is not included, a
simplification kept from the original simulator.

### Waypoint mission

`WaypointMission` holds waypoints $(\tau_j, p_j, \psi_j)$ with strictly increasing times. At time
$t$ the target is the last waypoint with $\tau_j < t$, or the first one if there is none. Times
count from the start of the node, hover included. The default mission in `config/params.yaml`:

| From $t$ [s] | Position [m] | Yaw [deg] |
| --- | --- | --- |
| 0 | (−10, 10, 10) | −90 |
| 22.5 | (−15, 10, 10) | 0 |
| 37.5 | (−15, 15, 10) | 0 |
| 52.5 | (−35, 15, 10) | 0 |
| 67.5 | (−45, 15, 7) | 0 |
| 82.5 | (−45, 15, 14) | 0 |
| 97.5 | (−45, −15, 14) | 0 |

### Outputs

| Output | Content |
| --- | --- |
| `state()` | Drone pose and velocity, published on `drone_odom_gt` |
| `imu()` | IMU in the LiDAR frame; its orientation with the drone position is published on `lidar_odom_gt` |
| `target()` | Active waypoint, shown by the `marker_setpoint` marker |

With the default mount (roll 15°, pitch 15°) the LiDAR's field of view is inclined relative to
the drone body.

## Time stepping

The ROS nodes call `step(t, dt)` at `update_rate` with $t$ the ROS time since the node started and
$\Delta t$ the time since the previous step (0 on the first step). They follow ROS time, so
`/use_sim_time` works, but $\Delta t$ depends on the actual loop timing and runs are not
bit-for-bit reproducible. The core classes are deterministic for a given sequence of $(t, \Delta t)$;
call them directly with a fixed $\Delta t$ for reproducible experiments or tests.
