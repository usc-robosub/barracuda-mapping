#!/usr/bin/bash

source /opt/ros/noetic/setup.bash
cd barracuda-mapping/catkin_ws
catkin_make
source devel/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-mapping/catkin_ws/devel/setup.bash" >> ~/.bashrc

roslaunch barracuda_mapping gtsam_slam.launch --wait
