#!/bin/bash

set -euo pipefail

echo "[entrypoint] Sourcing ROS Noetic..."
source /opt/ros/noetic/setup.bash

WORKDIR="/opt/barracuda-mapping/catkin_ws"
echo "[entrypoint] Changing to workspace: ${WORKDIR}"
cd "${WORKDIR}"

if [ ! -f src/CMakeLists.txt ]; then
  echo "[entrypoint] Initializing catkin workspace (src/CMakeLists.txt missing)"
  # In case this image is used without the repo-included file, still bootstrap
  catkin_init_workspace src || true
fi

echo "[entrypoint] Building workspace with catkin_make..."
catkin_make

echo "[entrypoint] Sourcing devel/setup.bash..."
source devel/setup.bash

{
  grep -q "/opt/ros/noetic/setup.bash" ~/.bashrc || echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  grep -q "/opt/barracuda-mapping/catkin_ws/devel/setup.bash" ~/.bashrc || echo "source /opt/barracuda-mapping/catkin_ws/devel/setup.bash" >> ~/.bashrc
} || true

echo "[entrypoint] Launching: roslaunch barracuda_mapping gtsam_slam.launch --wait"
exec roslaunch barracuda_mapping gtsam_slam.launch --wait
