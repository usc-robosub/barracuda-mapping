# Use the official ROS Noetic base image (based on Ubuntu 20.04)
FROM ros:noetic-ros-base

# Update and install SLAM-related packages (CUDA and glim)
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl vim wget git \
    # Install CUDA 12.2
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && sudo apt-get update \
    && apt-get install -y --no-install-recommends cuda-libraries-12-2 \
    # Install GLIM
    && curl -s --compressed "https://koide3.github.io/ppa/ubuntu2004/KEY.gpg" | gpg --dearmor | tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null \
    && echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2004 ./" | tee /etc/apt/sources.list.d/koide3_ppa.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends libiridescence-dev \
        libboost-all-dev \
        libglfw3-dev \
        libmetis-dev \
        libgtsam-points-cuda12.2-dev \
        ros-noetic-glim-ros-cuda12.2 \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-mapping

# Set working directory
WORKDIR /opt

# Source the ROS and workspace environments by default
CMD [ "/bin/bash", "/opt/barracuda-mapping/entrypoint.sh" ]
