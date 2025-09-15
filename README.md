# Barracuda Mapping

Lightweight ROS (Noetic) point‑cloud mapping node that aggregates incoming point clouds into a simple map and estimates odometry using either GTSAM iSAM2 pose graph or GLIM SLAM framework. Includes a basic collision‑check service against the accumulated map.

This repository provides:
- A catkin package `barracuda_mapping` with a C++ SLAM node `slam_node` (GTSAM-based).
- **NEW**: Integration with GLIM SLAM framework for enhanced underwater mapping.
- Launch files and YAML config for topics/frames with flexible SLAM backend selection.
- Optional Docker setup for a reproducible environment.

Note on dependencies: the project uses `liboctomap` and `octomap_msgs` directly and does not rely on `octomap_ros` or `octomap_server`. The Docker image and package manifest have been trimmed accordingly.

## Features
- **Dual SLAM Backend Support**: Choose between GTSAM iSAM2 (original) or GLIM SLAM (new).
- Incremental pose graph with GTSAM iSAM2 (prior + between factors).
- **GLIM Integration**: Advanced 3D SLAM with GPU acceleration support.
- Transforms incoming point clouds into `map` frame via TF2 and aggregates them.
- **Proper TF Tree**: Maintains correct `map -> odom -> base_link` transform hierarchy.
- Publishes odometry as `nav_msgs/Odometry`.
- Collision check service to test if a sphere intersects the current map.
- **Underwater Robot Optimized**: Special configurations for sonar and DVL sensors.

## Repo Layout
- `catkin_ws/src/barracuda_mapping/`
  - `src/slam_node.cpp`: Original GTSAM-based SLAM node implementation.
  - `launch/unified_mapping.launch`: **NEW** - Main launch file with SLAM backend selection.
  - `launch/gtsam_slam.launch`: Updated to use unified system (backwards compatible).
  - `launch/glim_barracuda.launch`: GLIM-specific configuration for Barracuda robot.
  - `config/gtsam_params.yaml`: Original GTSAM SLAM parameters.
  - `config/glim/`: **NEW** - GLIM configuration files optimized for underwater use.
  - `srv/CheckCollision.srv`: Service definition for collision checking.
  - `CMakeLists.txt`, `package.xml`: Build and dependencies.
- `catkin_ws/src/glim/`: **NEW** - GLIM core library (submodule).
- `catkin_ws/src/glim_ros1/`: **NEW** - GLIM ROS1 wrapper (submodule).
- `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`: Containerized build/run.

## Quickstart (Docker)

1) Build and run with docker compose:
```
docker compose up --build
```

This starts a container, builds the catkin workspace, sources the environment, and launches the unified mapping system with GLIM enabled by default.

**To use original GTSAM SLAM:**
```bash
# Modify docker-compose.yml or pass launch arguments:
docker compose run barracuda-mapping roslaunch barracuda_mapping gtsam_slam.launch use_glim:=false
```

Notes:
- The compose file uses `network_mode: host` to expose ROS topics directly.
- An NVIDIA GPU is reserved in the compose file for GLIM GPU acceleration (optional).
- GLIM requires additional dependencies (see GLIM Setup section below).

## SLAM Backend Selection

This repository now supports two SLAM backends:

### 1. GLIM SLAM (Default - Recommended)
**Features:**
- Advanced 3D LiDAR-IMU mapping framework
- GPU acceleration support 
- Superior accuracy and robustness
- Optimized for underwater environments

**Usage:**
```bash
# Default behavior - GLIM enabled
roslaunch barracuda_mapping unified_mapping.launch

# Explicitly enable GLIM
roslaunch barracuda_mapping unified_mapping.launch use_glim:=true
```

### 2. Original GTSAM SLAM (Legacy)
**Features:**
- Lightweight GTSAM iSAM2 implementation
- Lower computational requirements
- Proven stability

**Usage:**
```bash
# Disable GLIM to use original GTSAM
roslaunch barracuda_mapping unified_mapping.launch use_glim:=false

# Or use legacy launch file
roslaunch barracuda_mapping gtsam_slam.launch use_glim:=false
```

## GLIM Setup

For full GLIM functionality, additional dependencies are required:

### Prerequisites
- ROS Noetic
- GTSAM 4.3+ 
- Eigen3
- gtsam_points library

### TF Tree Structure (GLIM)
GLIM maintains the following TF tree for proper underwater robot operation:
```
map
 └── barracuda/odom
     └── barracuda/base_link
         └── barracuda/imu
             └── barracuda/sonar
```

Additional static transforms:
- `barracuda/base_link -> barracuda/dvl`

### Configuration
GLIM configurations are stored in `config/glim/` and optimized for:
- Sonar point cloud processing
- Underwater environment characteristics  
- Reduced computational load for embedded systems
- Proper frame naming for Barracuda robot

## Parameters
All parameters are loaded into the node’s private namespace via the launch file.

- `pointcloud_topics` (list|string): Input point cloud topic(s). Example:
  - List: `["left_camera/zed_point_cloud"]`
  - Single string also supported. Falls back to `/points` if unset.
- `odometry_topic` (string): Output odometry topic. Default: `slam/odometry`.
- `map_frame` (string): Map/world frame. Default: `map`.
- `base_frame` (string): Robot base frame. Default: `base_link` (overridden to `barracuda/base_link` in the YAML).
- `octomap_resolution` (double): OcTree resolution in meters per voxel. Default: `0.25`.
- `downsample_enabled` (bool): Enable VoxelGrid downsampling before OctoMap insertion. Default: `true`.
- `downsample_leaf_size` (double): VoxelGrid leaf size in meters. Defaults to `octomap_resolution` if unset. Default: `0.25` in the provided YAML.

See `config/gtsam_params.yaml` for an example configuration.

## Topics & Service
- Subscribed point cloud(s): from `~pointcloud_topics` (e.g., `left_camera/zed_point_cloud`).
- Published odometry: `~odometry_topic` (default `slam/odometry`) as `nav_msgs/Odometry`.
- Service: `check_collision` (`barracuda_mapping/CheckCollision`)
  - Request: `geometry_msgs/Point center`, `float64 radius`
  - Response: `bool collision`
- Published OctoMap (for RViz):
  - `octomap_full` (`octomap_msgs/Octomap`): Full map message (latched).
  - `octomap_binary` (`octomap_msgs/Octomap`): Binary map message (latched).

Example service call:
```
rosservice call /check_collision "center: {x: 0.0, y: 0.0, z: 0.0}  radius: 0.5"
```

## Frames
The node looks up TF from each cloud’s `frame_id` to `map_frame` at the message timestamp, and falls back to the latest available transform when necessary. Make sure your TF tree publishes `map_frame -> ... -> <cloud frame>` and `map_frame -> base_frame` as appropriate.

## Notes on Behavior
- Pose graph: starts with a prior at index 0, then adds a `BetweenFactor` between the last pose and the current TF-derived pose, updating iSAM2 incrementally.
- Map growth: transformed (optionally downsampled) points are appended to a single `pcl::PointCloud<pcl::PointXYZ>`. There is no loop‑closure map correction in this minimal example.
- Namespacing: `odometry_topic` without a leading `/` resolves relative to the node namespace (in the launch file it runs under `ns="barracuda"`).

## Customization
- Edit `config/gtsam_params.yaml` to change input topics and frames.
- Pass per‑launch overrides with `<param>` tags in a custom launch file.
- Add filtering/downsampling or loop‑closure logic to `src/slam_node.cpp` as needed.

## Troubleshooting
- No odometry output: verify TF is available between cloud frames and `map_frame`.
- No point clouds received: confirm `~pointcloud_topics` matches actual topic names.
- CMake cannot find octomap: ensure `liboctomap-dev` is installed in your environment. In Docker this is preinstalled; for native builds on Ubuntu 20.04/ROS Noetic: `sudo apt-get install liboctomap-dev ros-noetic-octomap-msgs`.
- **GLIM Issues**: See [GLIM Integration Guide](docs/GLIM_INTEGRATION.md) for GLIM-specific troubleshooting.
- **TF Tree Conflicts**: Ensure only one SLAM backend is active at a time.

## Documentation
- [GLIM Integration Guide](docs/GLIM_INTEGRATION.md) - Detailed GLIM setup and troubleshooting
- [Original GTSAM Documentation](#) - For legacy SLAM system

## Visualization
- RViz: Add an `Octomap` display and set Topic to `/barracuda/octomap_full` or `/barracuda/octomap_binary`. Fixed Frame should match your `map_frame` (default `map`).
- Note: `octomap_server` is not required here since the node publishes `octomap_msgs/Octomap` directly, which RViz can visualize without conversion.

## Native Setup (optional)
If you prefer building outside Docker on Ubuntu 20.04 with ROS Noetic:
- Install deps: `sudo apt-get update && sudo apt-get install -y build-essential libgtsam-dev liboctomap-dev ros-noetic-octomap-msgs ros-noetic-pcl-ros ros-noetic-tf2-eigen ros-noetic-tf2-sensor-msgs`
- Build: `cd catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make`
- Source: `source devel/setup.bash`
