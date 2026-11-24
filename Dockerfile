# Use the official ROS Noetic image based on Ubuntu 20.04
FROM ros:noetic-ros-base-focal

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Skip the ROS Noetic end-of-life dialog that RViz shows at startup
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Update and install necessary tools
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    libyaml-cpp-dev \
    python3-yaml \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-rosbridge-suite \
    ros-noetic-dynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

# Copy your package to the workspace
WORKDIR /root/catkin_ws/src
COPY . /root/catkin_ws/src/lidar_sim

# Build the workspace
WORKDIR /root/catkin_ws/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source the environment when starting a container
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the entry point to start a bash session with ROS environment sourced
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/.bashrc && bash"]

# Needed for running source
SHELL ["/bin/bash", "-c"]

# Reset DEBIAN_FRONTEND (optional but good practice)
ENV DEBIAN_FRONTEND=dialog