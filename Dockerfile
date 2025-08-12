# Use the official ROS Noetic base image (based on Ubuntu 20.04)
FROM ros:noetic-ros-base

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install minimal build and runtime dependencies
RUN apt-get update \ 
    && apt-get install -y --no-install-recommends \
        software-properties-common
RUN add-apt-repository ppa:borglab/gtsam-release-4.0
RUN apt-get install -y --no-install-recommends \
        build-essential \
        curl \
        vim \
        wget \
        git \
        libgtsam-dev \
        libgtsam-unstable-dev \
        ros-noetic-octomap \
        ros-noetic-octomap-msgs \
        ros-noetic-pcl-ros \
        ros-noetic-tf2-eigen \
        ros-noetic-tf2-sensor-msgs \
        ros-noetic-laser-pipeline \
    && rm -rf /var/lib/apt/lists/*

# Copy source code into container
COPY . /opt/barracuda-mapping

# Set working directory
WORKDIR /opt

# Run entrypoint on container start
CMD [ "/bin/bash", "/opt/barracuda-mapping/entrypoint.sh" ]
