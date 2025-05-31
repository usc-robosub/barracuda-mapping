# barracuda-mapping

## Octomap Server Parameters

### General Parameters
- **~frame_id** (string, default: `/map`):  
  Static global frame in which the map will be published. A transform from sensor data to this frame needs to be available when dynamically building maps.

- **~resolution** (float, default: `0.05`):  
  Resolution in meters for the map when starting with an empty map. Otherwise, the loaded file's resolution is used.

### Robot Parameters
- **~base_frame_id** (string, default: `base_footprint`):  
  The robot's base frame in which ground plane detection is performed (if enabled).

### Visualization Parameters
- **~height_map** (bool, default: `true`):  
  Whether visualization should encode height with different colors.

- **~color/[r/g/b/a]** (float):  
  Color for visualizing occupied cells when `~height_map=False`, in range `[0:1]`.

### Sensor Model Parameters
- **~sensor_model/max_range** (float, default: `-1` (unlimited)):  
  Maximum range in meters for inserting point cloud data when dynamically building a map. Limiting the range prevents spurious erroneous points far away from the robot.

- **~sensor_model/[hit|miss]** (float, default: `0.7 / 0.4`):  
  Probabilities for hits and misses in the sensor model when dynamically building a map.

- **~sensor_model/[min|max]** (float, default: `0.12 / 0.97`):  
  Minimum and maximum probability for clamping when dynamically building a map.

### Publishing Parameters
- **~latch** (bool, default: `True` for a static map, `false` if no initial map is given):  
  Whether topics are published latched or only once per change. For maximum performance when building a map (with frequent updates), set to `false`.

### Ground Filtering Parameters
- **~filter_ground** (bool, default: `false`):  
  Whether the ground plane should be detected and ignored from scan data when dynamically building a map, using `pcl::SACMODEL_PERPENDICULAR_PLANE`.

- **~ground_filter/distance** (float, default: `0.04`):  
  Distance threshold for points (in z direction) to be segmented to the ground plane.

- **~ground_filter/angle** (float, default: `0.15`):  
  Angular threshold of the detected plane from the horizontal plane to be detected as ground.

- **~ground_filter/plane_distance** (float, default: `0.07`):  
  Distance threshold from z=0 for a plane to be detected as ground (4th coefficient of the plane equation from PCL).

### Point Cloud Filtering Parameters
- **~pointcloud_[min|max]_z** (float, default: `-/+ infinity`):  
  Minimum and maximum height of points to consider for insertion in the callback. Points outside this interval will be discarded before insertion or ground plane filtering.

### Occupancy Filtering Parameters
- **~occupancy_[min|max]_z** (float, default: `-/+ infinity`):  
  Minimum and maximum height of occupied cells to be considered in the final map. This ignores all occupied voxels outside the interval when sending out visualizations and collision maps, but does not affect the actual octomap representation.

## Inputs
The Octomap server requires the following inputs:
- **Sensor Data**: Point cloud data from sensors such as LiDAR or depth cameras.
- **Frame ID**: The coordinate frame in which the map is generated (e.g., "world").
- **Parameters**: Configuration values such as resolution, max range, and occupancy thresholds, provided via the `octomap_server_params.yaml` file.

## Outputs
The Octomap server generates the following outputs:
- **Octomap**: A 3D occupancy grid map representing free and occupied spaces.
- **Visualization Topics**: ROS topics for visualizing the map in tools like RViz.
- **Map Compression**: Optionally compressed map data for efficient storage and transmission.