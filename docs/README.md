# lidar_sim documentation

lidar_sim simulates a spinning multi-beam LiDAR (Velodyne style) by casting rays against a scene
made of simple geometric primitives. Two motion models, a kinematic body on a circle and a
quadrotor flying waypoints, move the sensor around and publish IMU data and ground truth.

| Document | Contents |
| --- | --- |
| [Architecture](architecture.md) | Libraries, directory layout, classes, data flow, how to extend the code |
| [Conventions](conventions.md) | Frames, rotations, quaternions and units used everywhere |
| [Ray casting](ray_casting.md) | How a ray is intersected with the scene, from world to primitive and back |
| [Geometric primitives](primitives.md) | Math of the plane, sphere, cylinder, cone and ellipsoid intersections |
| [Plane constraints](constraints.md) | Math of the constraints that cut primitives, with recipes |
| [LiDAR model](lidar_model.md) | Beam layout, scan timing, range gating and the point cloud format |
| [Motion models](motion_models.md) | Rigid body integration, IMU model, circle tracking and quadrotor control |
| [Configuration](configuration.md) | World file, parameters, the standalone viewer, launch arguments, world generator |
| [ROS interface](ros_interface.md) | Nodes, topics, frames and TF |

New to the code? Read [Architecture](architecture.md) first, then [Conventions](conventions.md).
