# Depth Camera to Laser Scan Setup

This package now includes support for converting depth camera images to laser scan data for SLAM operations.

## Components Added

1. **depthimage_to_laserscan** ROS package dependency
2. **Configuration file**: `config/depthimage_to_laserscan_params.yaml`
3. **Launch files**: 
   - Updated `slam_launcher.launch` with depth-to-scan conversion
   - New `depth_to_scan.launch` for standalone conversion

## Usage

### Full SLAM with Depth Camera
Run the complete SLAM system with depth camera conversion:
```bash
roslaunch barracuda_mapping slam_launcher.launch
```

### Depth Conversion Only
To run just the depth image to laser scan conversion:
```bash
roslaunch barracuda_mapping depth_to_scan.launch
```

## Topic Configuration

The system expects the following topics from your depth camera:
- **Input**: `/camera/depth/image_raw` (depth image)
- **Input**: `/camera/depth/camera_info` (camera calibration)
- **Output**: `/scan` (laser scan data for SLAM)

### Customizing Camera Topics

If your camera publishes to different topics, modify the remapping in the launch files:
```xml
<remap from="image" to="/your_camera/depth/image_raw" />
<remap from="camera_info" to="/your_camera/depth/camera_info" />
```

## Configuration Parameters

Key parameters in `config/depthimage_to_laserscan_params.yaml`:
- `range_min/max`: Distance range in meters (0.45m to 10.0m)
- `angle_min/max`: Angular field of view in radians (±90 degrees)
- `angle_increment`: Angular resolution (~0.5 degrees)
- `scan_height`: Number of pixel rows used for scan generation

## Docker Build

The Docker setup includes the necessary ROS packages:
```bash
docker-compose up --build
```