# GLIM Integration for Barracuda Mapping

## Overview

This document describes the integration of GLIM (3D LiDAR-IMU Mapping Framework) with the Barracuda underwater robot mapping system.

## Key Issues Addressed

### TF Tree Conflicts
**Problem**: Multiple SLAM systems publishing conflicting TF transforms.
**Solution**: 
- GLIM publishes: `map -> barracuda/odom -> barracuda/base_link`
- Original SLAM has `publish_tf: false` to avoid conflicts
- Static transforms handle sensor relationships

### Frame Naming Convention
**Problem**: Inconsistent frame naming between GLIM and Barracuda robot.
**Solution**:
```
map (world frame)
└── barracuda/odom (odometry frame)
    └── barracuda/base_link (robot base)
        ├── barracuda/imu (IMU sensor)
        │   └── barracuda/sonar (primary mapping sensor)
        └── barracuda/dvl (DVL sensor)
```

### Underwater Sensor Integration
**Problem**: GLIM designed for LiDAR, Barracuda uses sonar.
**Solution**:
- Sonar pointclouds treated as "lidar" data in GLIM
- Optimized preprocessing for sonar characteristics
- Adjusted filtering and downsampling parameters

## Configuration Files

### Main Configuration
- `config/glim/config.json` - Master configuration selector
- `config/glim/config_ros.json` - ROS topics, frames, and TF settings

### Sensor-Specific
- `config/glim/config_preprocess.json` - Sonar data preprocessing
- `config/glim/config_odometry_cpu.json` - Odometry estimation
- `config/glim/config_sensors.json` - Sensor calibration

## Launch Files

### Primary Launch Files
```bash
# Use GLIM SLAM (recommended)
roslaunch barracuda_mapping unified_mapping.launch use_glim:=true

# Use original GTSAM SLAM
roslaunch barracuda_mapping unified_mapping.launch use_glim:=false

# Legacy compatibility
roslaunch barracuda_mapping gtsam_slam.launch use_glim:=false
```

### Specialized Launch Files
- `glim_barracuda.launch` - GLIM-only configuration
- `unified_mapping.launch` - Main launch file with backend selection

## Troubleshooting

### TF Tree Issues
**Symptom**: "Could not find transform" errors
**Check**:
1. Verify static transforms are published: `rosrun tf view_frames`
2. Check frame names match configuration
3. Ensure only one SLAM system publishes map->odom

### GLIM Build Issues  
**Symptom**: GLIM nodes fail to start
**Solution**:
1. Ensure all submodules are initialized: `git submodule update --init --recursive`
2. Check GTSAM version compatibility
3. Verify gtsam_points library installation

### Sonar Data Processing
**Symptom**: Poor mapping quality with sonar data
**Adjustments**:
1. Modify `config_preprocess.json` for sonar characteristics
2. Adjust outlier removal parameters
3. Tune voxel grid resolution

## Performance Optimization

### CPU vs GPU
- Default configuration uses CPU modules for compatibility
- GPU acceleration available by changing config files to `*_gpu.json`

### Memory Usage
- `keep_raw_points: false` to reduce memory
- Adjust `random_downsample_target` for point density control

### Real-time Performance
- Reduce `random_downsample_target` if processing too slow
- Disable advanced features for embedded systems

## Integration Testing

### Verification Steps
1. **TF Tree Structure**:
   ```bash
   rosrun tf view_frames
   # Should show proper hierarchy without conflicts
   ```

2. **Topic Flow**:
   ```bash
   rostopic echo /barracuda/slam/odometry
   rostopic echo /barracuda/slam/pose
   ```

3. **Visualization**:
   ```bash
   rosrun rviz rviz -d $(rospack find glim_ros)/rviz/glim_ros.rviz
   ```

## Known Limitations

1. **GLIM Dependencies**: Requires additional libraries not included in minimal Docker
2. **GPU Support**: Full GPU acceleration needs CUDA setup
3. **Calibration**: Sensor calibration files may need adjustment for specific hardware

## Future Improvements

1. **Automatic Sensor Detection**: Detect sonar vs LiDAR automatically
2. **Dynamic Configuration**: Runtime switching between SLAM backends
3. **Extended Sensor Support**: Integration with additional underwater sensors
4. **Performance Monitoring**: Built-in performance metrics and tuning guides