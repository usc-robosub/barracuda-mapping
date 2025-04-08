# Use the official ROS Noetic base image (based on Ubuntu 20.04)
FROM ros:noetic-ros-base

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install SLAM-related packages (e.g. gmapping and navigation)
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim git \
    ros-noetic-slam-toolbox \
    ros-noetic-navigation \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-mapping

# Set working directory
WORKDIR /opt

# Source the ROS and workspace environments by default
CMD [ "/bin/bash", "/opt/barracuda-mapping/entrypoint.sh" ]
