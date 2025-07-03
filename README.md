# barracuda-mapping

ROS-based SLAM mapping system for underwater robotics, supporting both traditional LiDAR and depth camera inputs.

## Features

- **SLAM Toolbox integration** for real-time mapping
- **Depth camera support** via depthimage_to_laserscan conversion
- **Docker containerization** for easy deployment
- **Configurable parameters** for different sensor setups

## Quick Start

```bash
# Build and run with Docker
docker-compose up --build

# Or run directly in ROS environment
roslaunch barracuda_mapping slam_launcher.launch
```

## Documentation

- [Depth Camera Setup Guide](DEPTH_CAMERA_SETUP.md) - How to configure depth cameras for mapping

## Requirements

- ROS Noetic
- SLAM Toolbox
- depthimage_to_laserscan package