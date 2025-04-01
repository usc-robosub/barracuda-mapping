# Use the official ROS Noetic base image (based on Ubuntu 20.04)
FROM ros:noetic-ros-base

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install SLAM-related packages (e.g. gmapping and navigation)
RUN apt-get update && apt-get install -y \
    ros-noetic-slam-toolbox \
    ros-noetic-navigation \
    && rm -rf /var/lib/apt/lists/*

# Create a Catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Initialize the workspace (optional if you have custom packages)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Optionally, copy your own ROS packages into the workspace
# COPY ./your_package /catkin_ws/src/your_package

# Source the ROS and workspace environments by default
CMD [ "bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && bash" ]
