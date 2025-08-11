# Barracuda Mapping

Lightweight ROS (Noetic) point‑cloud mapping node that aggregates incoming point clouds into a simple map and estimates odometry using a GTSAM iSAM2 pose graph. Includes a basic collision‑check service against the accumulated map.

This repository provides:
- A catkin package `barracuda_mapping` with a single C++ node `slam_node`.
- A launch file and YAML config for topics/frames.
- Optional Docker setup for a reproducible environment.

Note on dependencies: the project uses `liboctomap` and `octomap_msgs` directly and does not rely on `octomap_ros` or `octomap_server`. The Docker image and package manifest have been trimmed accordingly.

## Features
- Incremental pose graph with GTSAM iSAM2 (prior + between factors).
- Transforms incoming point clouds into `map` frame via TF2 and aggregates them.
- Publishes odometry as `nav_msgs/Odometry`.
- Collision check service to test if a sphere intersects the current map.

## Repo Layout
- `catkin_ws/src/barracuda_mapping/`
  - `src/slam_node.cpp`: Main node implementation.
  - `launch/gtsam_slam.launch`: Launches the node and loads parameters.
  - `config/gtsam_params.yaml`: Default parameters (topics, frames, output topic).
  - `srv/CheckCollision.srv`: Service definition for collision checking.
  - `CMakeLists.txt`, `package.xml`: Build and dependencies.
- `Dockerfile`, `docker-compose.yml`, `entrypoint.sh`: Containerized build/run.

## Quickstart (Docker)

1) Build and run with docker compose:
```
docker compose up --build
```

This starts a container, builds the catkin workspace, sources the environment, and launches `gtsam_slam.launch`.

Notes:
- The compose file uses `network_mode: host` to expose ROS topics directly.
- An NVIDIA GPU is reserved in the compose file, but the node itself does not require GPU.

The launch file loads defaults from `config/gtsam_params.yaml`.

## Parameters
All parameters are loaded into the node’s private namespace via the launch file.

- `pointcloud_topics` (list|string): Input point cloud topic(s). Example:
  - List: `["left_camera/zed_point_cloud"]`
  - Single string also supported. Falls back to `/points` if unset.
- `odometry_topic` (string): Output odometry topic. Default: `slam/odometry`.
- `map_frame` (string): Map/world frame. Default: `map`.
- `base_frame` (string): Robot base frame. Default: `base_link` (overridden to `barracuda/base_link` in the YAML).

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
- Map growth: all transformed points are appended to a single `pcl::PointCloud<pcl::PointXYZ>`. There is no downsampling or loop‑closure map correction in this minimal example.
- Namespacing: `odometry_topic` without a leading `/` resolves relative to the node namespace (in the launch file it runs under `ns="barracuda"`).

## Customization
- Edit `config/gtsam_params.yaml` to change input topics and frames.
- Pass per‑launch overrides with `<param>` tags in a custom launch file.
- Add filtering/downsampling or loop‑closure logic to `src/slam_node.cpp` as needed.

## Troubleshooting
- No odometry output: verify TF is available between cloud frames and `map_frame`.
- No point clouds received: confirm `~pointcloud_topics` matches actual topic names.
- CMake cannot find octomap: ensure `liboctomap-dev` is installed in your environment. In Docker this is preinstalled; for native builds on Ubuntu 20.04/ROS Noetic: `sudo apt-get install liboctomap-dev ros-noetic-octomap-msgs`.

## Visualization
- RViz: Add an `Octomap` display and set Topic to `/barracuda/octomap_full` or `/barracuda/octomap_binary`. Fixed Frame should match your `map_frame` (default `map`).
- Note: `octomap_server` is not required here since the node publishes `octomap_msgs/Octomap` directly, which RViz can visualize without conversion.

## Native Setup (optional)
If you prefer building outside Docker on Ubuntu 20.04 with ROS Noetic:
- Install deps: `sudo apt-get update && sudo apt-get install -y build-essential libgtsam-dev liboctomap-dev ros-noetic-octomap-msgs ros-noetic-pcl-ros ros-noetic-tf2-eigen ros-noetic-tf2-sensor-msgs`
- Build: `cd catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make`
- Source: `source devel/setup.bash`
