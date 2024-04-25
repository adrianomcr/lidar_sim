# lidar_sim

Simulation of a LiDAR based on geometric primitives. Below an example for the Velodyne VLP16 configuration.

![image](.images/example.png)

## Set it up
```bash
mkdir ~/simulation_ws/src
cd ~/simulation_ws/src
git clone git@github.com:adrianomcr/lidar_sim.git
cd ~/simulation_ws
catkin build  --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

OBS: The `Release` argument is important to make the code run (way) faster.


## Use it
```bash
cd ~/simulation_ws
source devel/setup.bash
roslaunch lidar_sim example.launch
```

## About the software

Each object in the scene is composed of one basif geometric shape (geometric primitives). Its is also possible to use a constraint to "cut" of part of the geometric primitive.


### Available geometric primitives

- Plane
- Sphere
- Cylinder
- Ellipsoid

### Available constraints

- Plane

## Contact

Adriano Rezende

adrianomcr18@gmail.com