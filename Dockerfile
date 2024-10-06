# Use the official ROS Noetic image based on Ubuntu 20.04
FROM ros:noetic-ros-base-focal

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-rosbridge-suite \
    ros-noetic-dynamic-reconfigure \
    && rm -rf /var/lib/apt/lists/*

# Install any other dependencies (if required, adjust based on your package)
# RUN apt-get install -y ros-noetic-<other-packages>

# Copy your package to the workspace
WORKDIR /root/catkin_ws/src
COPY . /root/catkin_ws/src/lidar_sim

# Build the workspace
WORKDIR /root/catkin_ws/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Source the environment when starting a container
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the entry point to start a bash session with ROS environment sourced
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source ~/.bashrc && bash"]

# Needed for running source
SHELL ["/bin/bash", "-c"]
