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
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-tf2-ros \
        libeigen3-dev \
        libnanoflann-dev \
    && rm -rf /var/lib/apt/lists/*

# Note: For full GLIM support, additional dependencies may be required:
# - GTSAM 4.3+ (custom build may be needed)
# - gtsam_points library
# - CUDA (for GPU acceleration)
# See docs/GLIM_SETUP.md for complete installation instructions

# Copy source code into container
COPY . /opt/barracuda-mapping

# Set working directory
WORKDIR /opt

# Run entrypoint on container start
CMD [ "/bin/bash", "/opt/barracuda-mapping/entrypoint.sh" ]
