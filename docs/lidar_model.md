# LiDAR model

`LidarScanner` (`include/lidar_sim/lidar_scanner.h`) models a spinning multi-beam LiDAR: a column
of $N_r$ beams (rings) fired $N_a$ times per revolution at evenly spaced azimuths.

## Parameters

| Symbol | `LidarConfig` field | Parameter in `params.yaml` | Default |
| --- | --- | --- | --- |
| $f$ | `scan_rate` | `scan_rate` | 10 Hz |
| $N_a$ | `num_azimuth_steps` | `num_azimuth_steps` | 1024 (0.35°) |
| $N_r$ | `num_rings` | `num_rings` | 64 |
| $e_{\min}$ | `min_elevation` [rad] | `min_elevation_deg` | −45° |
| $e_{\max}$ | `max_elevation` [rad] | `max_elevation_deg` | 45° |
| $r_{\min}$, $r_{\max}$ | `min_range`, `max_range` | same | 1 m, 100 m |
| | `intensity` | `intensity` | 60 |
| | `num_threads` | `num_threads` | 8 |

The shipped example uses 1024 azimuth steps and 64 rings from −45° to +45°. The original
simulator used 16 rings from −22.5° to 19.6875°, spaced by 2.8125°. A real Velodyne VLP-16
has 16 rings from −15° to +15° (2° spacing); set `num_rings: 16`, `min_elevation_deg: -15` and
`max_elevation_deg: 15` to match it.

The constructor rejects inconsistent values (non-positive rate, fewer than one ring or step,
$e_{\max} < e_{\min}$, $r_{\max} \le r_{\min}$) and clamps `num_threads` to $[1, N_r]$.

## Beam geometry

Ring $k \in \lbrace 0, \dots, N_r - 1 \rbrace$ points at the elevation

```math
e_k = e_{\min} + k\, \Delta e, \qquad \Delta e = \frac{e_{\max} - e_{\min}}{N_r - 1}
```

($\Delta e = 0$ for a single ring), so ring 0 is the lowest beam. Firing
$i \in \lbrace 0, \dots, N_a - 1 \rbrace$ happens at the azimuth

```math
\alpha_i = \frac{2\pi\, i}{N_a}.
```

The unit direction of beam $(i, k)$ in the sensor frame is

```math
b_{i,k} = \begin{bmatrix} \cos e_k \cos\alpha_i \\ -\cos e_k \sin\alpha_i \\ \sin e_k \end{bmatrix}.
```

The minus sign makes the head spin **clockwise seen from above**, like a Velodyne: azimuth 0 is
the sensor's $+x$ axis and azimuth grows towards $-y$. The azimuth of any point $p$ of the cloud
is therefore $\mathrm{atan2}(-p_y, p_x)$, wrapped to $[0, 2\pi)$.

All directions, azimuths and firing times are computed once in the constructor.

## One revolution

`scan(world, position, orientation, returns)` simulates a full revolution with the sensor frozen
at the pose $(x, R)$:

1. For each beam, cast the world ray $o = x$, $d = R\thinspace b_{i,k}$ and get the closest distance
   $t^\star$ ([Ray casting](ray_casting.md)).
2. Keep the return only if $r_{\min} < t^\star < r_{\max}$. Misses ($t^\star = \infty$) and hits
   outside the range are marked invalid.
3. Express the hit in the sensor frame. The ray is rigidly attached to the sensor, so this is
   simply

```math
p_{i,k} = R^\top \big( (x + t^\star R\, b_{i,k}) - x \big) = t^\star\, b_{i,k}.
```

Each `LidarReturn` stores `valid`, the point, `distance` $= t^\star$, `azimuth` $= \alpha_i$, the
constant `intensity`, `ring` $= k$, and `time` (below). Returns are stored at index
$i \cdot N_r + k$: firing first, ring second.

## Timing

With the scan period $T = 1/f$, firing $i$ happens at

```math
\tau_i = T\, \frac{i}{N_a}
```

after the start of the revolution, and all rings of a firing share that time. With the defaults
the times go from 0 to 99.94 ms.

The geometry of a scan uses a **single pose** for the whole revolution: the simulator does not
reproduce the motion distortion of a sensor moving while it spins, even though the `time` field
describes the sweep. An algorithm that deskews clouds from the `time` field and the sensor motion
(IMU or odometry) will correct a distortion that is not there; disable deskewing when feeding it
simulated clouds.

In the ROS node, the pose is the latest one received on `lidar_odom_gt` when the computation
starts. The node then waits for the end of the scan period and publishes the cloud and the
`world -> tf_child_frame` TF with the same stamp and the same pose. With the default
`tf_child_frame` (`lidar`) the cloud is consistent with TF. The stamp is the publication time,
about one scan period after the pose was sampled.

## Point cloud format

`toPointCloud2()` (`ros/conversions.cpp`) writes an unorganized cloud (`height` 1) in the
Velodyne driver layout, with a point step of 48 bytes:

| Field | Type | Offset | Meaning |
| --- | --- | --- | --- |
| `x`, `y`, `z` | float32 | 0, 4, 8 | Point in the `lidar` frame [m] |
| | | 12 | 4 bytes of padding |
| `intensity` | float32 | 16 | Constant `intensity` parameter |
| `azimuth` | float32 | 20 | $\alpha_i$ [rad], in $[0, 2\pi)$ |
| `distance` | float32 | 24 | Range $t^\star$ [m] |
| `ring` | uint16 | 28 | Beam index $k$, 0 is the lowest |
| | | 30 | 2 bytes of padding |
| `time` | float64 | 32 | $\tau_i$ [s] since the start of the revolution |
| `return_number` | uint8 | 40 | Always 1 (single return) |
| | | 41 | 7 bytes of padding |

- The `fields` array lists them in the order x, y, z, intensity, time, ring, azimuth, distance,
  return_number; consumers should look fields up by name, not by position.
- Only valid returns are written, in index order (by firing, then by ring). `width` is the number
  of valid returns, `is_dense` is true (no NaN points), and a scan without hits gives an empty
  cloud.
- Padding bytes are zero.

`investigate` prints a few points of every cloud with all their fields, which is handy to check
this layout against a consumer.

## Threads and performance

The rings are split into `num_threads` contiguous blocks; thread $j$ processes rings
$\lfloor j N_r / T_h \rfloor$ to $\lfloor (j+1) N_r / T_h \rfloor - 1$, where $T_h$ is the
number of threads. Each thread writes only its own returns, so no locking is needed.

The work per revolution is $N_a N_r$ rays times the number of objects (about 3.7 million
ray-primitive tests with the defaults and the example scene). The node logs a throttled warning
when a scan takes longer than the scan period, which means the published rate will drop below
`scan_rate`. Reduce `num_azimuth_steps`, the number of objects, or the rate, or add threads.

## What is not modeled

The sensor is ideal. There is no range noise, no reflectivity-dependent intensity, no beam
divergence or multiple returns, no dropouts, and no motion distortion within a revolution. The
core library is the place to add them: everything happens in `LidarScanner::scanRings()`.
