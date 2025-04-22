ARG CUDA=12-2
FROM nvcr.io/nvidia/l4t-base:35.4.1

RUN apt-get update \
    && apt-get install -y --no-install-recommends build-essential curl git vim wget \
    # Install CUDA 12.2
    && wget -q https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/cuda-ubuntu2004.pin \
    && mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget -q https://developer.download.nvidia.com/compute/cuda/12.2.2/local_installers/cuda-tegra-repo-ubuntu2004-12-2-local_12.2.2-1_arm64.deb \
    && dpkg -i cuda-tegra-repo-ubuntu2004-12-2-local_12.2.2-1_arm64.deb \
    && cp /var/cuda-tegra-repo-ubuntu2004-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
    && apt-get update \
    && apt-get -y install cuda-toolkit \
    # Install ROS
    && echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-noetic-ros-base \
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
