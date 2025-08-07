# Use the official ROS Noetic base image (based on Ubuntu 20.04)
FROM ros:noetic-ros-base

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Preseed keyboard configuration to prevent interactive prompt
RUN echo 'keyboard-configuration keyboard-configuration/layout select English (US)' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/modelcode string pc105' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/variant select US' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/layoutcode string us' | debconf-set-selections

# Install dependencies and CUDA runtime
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        curl \
        vim \
        wget \
        git \
        gnupg \
        ca-certificates \
    # Install NVIDIA CUDA keyring
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update \
    # Install CUDA runtime (not full toolkit)
    && apt-get install -y --no-install-recommends cuda-runtime-12-2 \
    # Add Koide3 PPA for GLIM
    && curl -s --compressed "https://koide3.github.io/ppa/ubuntu2004/KEY.gpg" | gpg --dearmor | tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg > /dev/null \
    && echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2004 ./" | tee /etc/apt/sources.list.d/koide3_ppa.list \
    && apt-get update \
    # Install GLIM + SLAM-related dependencies
    && apt-get install -y --no-install-recommends \
        libiridescence-dev \
        libboost-all-dev \
        libglfw3-dev \
        libmetis-dev \
        libgtsam-points-cuda12.2-dev \
        ros-noetic-glim-ros-cuda12.2 \
    # Clean up
    && rm -rf /var/lib/apt/lists/*

# Copy source code into container
COPY . /opt/barracuda-mapping

# Set working directory
WORKDIR /opt

# Run entrypoint on container start
CMD [ "/bin/bash", "/opt/barracuda-mapping/entrypoint.sh" ]

