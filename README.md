# Lidar Processing

A ROS2 workspace containing packages for processing 3D LiDAR pointcloud data and converting it to 2D laser scans for navigation and SLAM applications.

## Overview

This workspace provides a complete pipeline for:
1. **Simulating** LiDAR data for testing
2. **Downsampling** dense 3D pointclouds to reduce computational load
3. **Converting** 3D pointclouds to 2D laser scans
4. **Pipeline integration** combining downsampling and conversion in one launch
5. **SLAM Toolbox** Running SLAM Toolbox on the 2D scan for mapping and localization

## Architecture

```
┌─────────────────┐     ┌──────────────────────┐     ┌─────────────────────┐     ┌───────────┐     ┌───────────────┐
│  LiDAR Sensor   │────▶│ Pointcloud           │────▶│ Pointcloud to       │────▶│ 2D Laser  │────▶│ SLAM Toolbox  │
│  /ouster/points │     │ Downsampling         │     │ Laserscan           │     │ /scan     │     │ /map          │
│                 │     │ /downsampled_points  │     │                     │     │           │     │               │
└─────────────────┘     └──────────────────────┘     └─────────────────────┘     └───────────┘     └───────────────┘
```

## Packages

### 1. lidar_simulator

Simulates LiDAR pointcloud data for testing and development without physical hardware.

#### Features
- Generates synthetic 3D pointcloud data
- Configurable scan patterns and noise
- Useful for testing downstream processing nodes

#### Launch
```bash
ros2 launch lidar_simulator lidar_simulator.launch.py
```

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `frame_id` | `lidar_link` | Frame ID for the simulated pointcloud |
| `publish_rate` | `10.0` | Publishing rate in Hz |
| `num_points` | `10000` | Number of points per scan |

---

### 2. pointcloud_downsampling

Reduces dense 3D pointcloud data to a manageable size while preserving essential geometric information.

#### Features
- **Voxel Grid** downsampling - divides space into voxels and keeps one point per voxel
- **Uniform** downsampling - keeps every Nth point
- **Random** downsampling - randomly samples a percentage of points
- Configurable maximum points limit

#### Launch
```bash
# Basic launch
ros2 launch pointcloud_downsampling downsampler.launch.py

# With custom parameters
ros2 launch pointcloud_downsampling downsampler.launch.py \
    input_topic:=/velodyne_points \
    output_topic:=/downsampled_points \
    downsample_method:=voxel \
    voxel_size:=0.1
```

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/ouster/points` | Input pointcloud topic |
| `output_topic` | `/downsampled_points` | Output downsampled pointcloud topic |
| `downsample_method` | `voxel` | Method: `voxel`, `uniform`, `random` |
| `voxel_size` | `0.1` | Voxel size in meters (for voxel method) |
| `uniform_skip` | `10` | Skip factor (for uniform method) |
| `random_ratio` | `0.1` | Keep ratio 0.0-1.0 (for random method) |
| `max_points` | `50000` | Maximum output points |

#### Run Node Directly
```bash
ros2 run pointcloud_downsampling pointcloud_downsampler \
    --ros-args \
    -p input_topic:=/ouster/points \
    -p output_topic:=/downsampled_points \
    -p downsample_method:=voxel \
    -p voxel_size:=0.05
```

---

### 3. pointcloud_to_laserscan

Converts 3D pointcloud data to 2D laser scan messages by projecting points within a specified height range.

#### Features
- Configurable height range for point selection
- TF2 transform support for frame conversion
- Adjustable laser scan angular resolution
- Compatible with navigation stack (Nav2)

#### Launch
```bash
# Basic launch
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py

# With custom parameters
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py \
    scanner:=my_scanner
```

#### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `sub_topic` | `cloud_in` | Input pointcloud topic |
| `pub_topic` | `scan` | Output laser scan topic |
| `target_frame` | `` | Target TF frame (empty = use cloud frame) |
| `transform_tolerance` | `0.01` | TF transform tolerance in seconds |
| `min_height` | `-inf` | Minimum height in meters |
| `max_height` | `inf` | Maximum height in meters |
| `angle_min` | `-π` | Minimum scan angle in radians |
| `angle_max` | `π` | Maximum scan angle in radians |
| `angle_increment` | `π/180` | Angular resolution in radians |
| `scan_time` | `0.033` | Time between scans in seconds |
| `range_min` | `0.0` | Minimum range in meters |
| `range_max` | `inf` | Maximum range in meters |
| `use_inf` | `true` | Use infinity for out-of-range values |
| `queue_size` | `auto` | Input queue size |

#### Run Node Directly
```bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -p sub_topic:=/downsampled_points \
    -p pub_topic:=/scan \
    -p target_frame:=base_link \
    -p min_height:=-0.1 \
    -p max_height:=0.5
```

---

### 4. pcl_to_scan_pipeline

Complete pipeline that combines pointcloud downsampling, laser scan conversion, and SLAM in a single launch file.

#### Features
- Single launch for the complete processing chain
- Configurable parameters for both stages
- Optional static TF publisher for testing
- SLAM Toolbox integration for mapping
- Ready for integration with Nav2

#### Launch Files

##### Basic Pipeline (Downsampling + Laserscan)
```bash
ros2 launch pcl_to_scan_pipeline pcl_to_scan_pipeline.launch.py

# With custom parameters
ros2 launch pcl_to_scan_pipeline pcl_to_scan_pipeline.launch.py \
    input_topic:=/ouster/points \
    output_scan_topic:=/scan \
    voxel_size:=0.05 \
    min_height:=-0.2 \
    max_height:=0.3
```

##### SLAM Toolbox Pipeline
```bash
# Launch with SLAM Toolbox for mapping
ros2 launch pcl_to_scan_pipeline slam_toolbox.launch.py
```

#### Parameters (pcl_to_scan_pipeline)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/ouster/points` | Input dense pointcloud topic |
| `downsampled_topic` | `/downsampled_points` | Intermediate topic |
| `output_scan_topic` | `/scan` | Output laser scan topic |
| `target_frame` | `base_link` | Target frame for laser scan |
| `downsample_method` | `voxel` | Downsampling method |
| `voxel_size` | `0.05` | Voxel size in meters |
| `min_height` | `-0.1` | Min height for scan slice |
| `max_height` | `0.5` | Max height for scan slice |
| `range_min` | `0.1` | Minimum scan range |
| `range_max` | `100.0` | Maximum scan range |

#### SLAM Toolbox Configuration

The SLAM Toolbox parameters are configured in `config/mapper_params_online_async.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `odom_frame` | `os_sensor` | Odometry frame (use sensor frame if no odom) |
| `map_frame` | `map` | Map frame |
| `base_frame` | `os_sensor` | Robot base frame |
| `scan_topic` | `/scan` | Input laser scan topic |
| `mode` | `mapping` | SLAM mode: `mapping` or `localization` |
| `resolution` | `0.05` | Map resolution in meters |
| `max_laser_range` | `20.0` | Maximum laser range |
| `minimum_travel_distance` | `0.0` | Min distance before adding new scan |
| `minimum_travel_heading` | `0.0` | Min rotation before adding new scan |

---

## SLAM Toolbox Usage

### Running SLAM with a ROS Bag

```bash
# Terminal 1: Play the rosbag with clock
ros2 bag play /path/to/your/bag --clock --rate 0.3 --topics /ouster/points

# Terminal 2: Launch the SLAM pipeline
ros2 launch pcl_to_scan_pipeline slam_toolbox.launch.py

# Terminal 3: Visualize in RViz2
rviz2
# Add displays for:
# - Map (/map)
# - LaserScan (/scan)
# - TF
```

### Saving Maps

#### Save Occupancy Grid Map (Standard Format)
```bash
# Create maps directory
mkdir -p /ros_ws/maps

# Save map using nav2_map_server
ros2 run nav2_map_server map_saver_cli -f /ros_ws/maps/my_map

# This creates:
# /ros_ws/maps/my_map.pgm  - The map image
# /ros_ws/maps/my_map.yaml - The map metadata
```

#### Save Serialized Map (For Resuming SLAM)
```bash
# Save pose graph that can be loaded later
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/ros_ws/maps/my_map'}"

# This creates:
# /ros_ws/maps/my_map.posegraph - Serialized pose graph
# /ros_ws/maps/my_map.data      - Serialized map data
```

### Output Files
| File | Description |
|------|-------------|
| `my_map.pgm` | Occupancy grid image |
| `my_map.yaml` | Map metadata (resolution, origin) |
| `my_map.posegraph` | Serialized pose graph (slam_toolbox) |
| `my_map.data` | Serialized map data (slam_toolbox) |

---

## Installation

### Prerequisites
- ROS2 Humble
- PCL (Point Cloud Library)
- TF2
- SLAM Toolbox
- Nav2 Map Server (for saving maps)

### Install Dependencies
```bash
sudo apt update
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-tf2-ros \
    ros-humble-tf2-sensor-msgs
```

### Build
```bash
cd /ros_ws
colcon build --packages-select \
    lidar_simulator \
    pointcloud_downsampling \
    pointcloud_to_laserscan \
    pcl_to_scan_pipeline

source install/setup.bash
```

### Build Single Package
```bash
colcon build --packages-select pcl_to_scan_pipeline
source install/setup.bash
```

---

## Usage Examples

### Example 1: Process Real LiDAR Data
```bash
# Terminal 1: Run the pipeline
ros2 launch pcl_to_scan_pipeline pcl_to_scan_pipeline.launch.py \
    input_topic:=/ouster/points

# Terminal 2: Visualize in RViz2
rviz2
# Add LaserScan display for /scan topic
# Add PointCloud2 display for /downsampled_points topic
```

### Example 2: Test with Simulated Data
```bash
# Terminal 1: Launch simulator
ros2 launch lidar_simulator lidar_simulator.launch.py

# Terminal 2: Launch processing pipeline
ros2 launch pcl_to_scan_pipeline pcl_to_scan_with_tf.launch.py \
    input_topic:=/simulated_pointcloud
```

### Example 3: Downsampling Only
```bash
# Reduce pointcloud density for visualization or other processing
ros2 launch pointcloud_downsampling downsampler.launch.py \
    input_topic:=/ouster/points \
    output_topic:=/sparse_cloud \
    voxel_size:=0.2
```

### Example 4: Integration with Nav2
```bash
# Launch pipeline with Nav2-compatible output
ros2 launch pcl_to_scan_pipeline pcl_to_scan_pipeline.launch.py \
    output_scan_topic:=/scan \
    target_frame:=base_link \
    min_height:=0.0 \
    max_height:=0.5 \
    range_max:=25.0
```

### Example 5: Complete SLAM Pipeline with Rosbag
```bash
# Terminal 1: Play rosbag
ros2 bag play /path/to/bag --clock --rate 0.3 --topics /ouster/points

# Terminal 2: Launch SLAM
ros2 launch pcl_to_scan_pipeline slam_toolbox.launch.py

# Terminal 3: Save map when done
ros2 run nav2_map_server map_saver_cli -f /ros_ws/maps/my_map
```

---

## Topics

### Published Topics
| Topic | Type | Package | Description |
|-------|------|---------|-------------|
| `/downsampled_points` | `sensor_msgs/PointCloud2` | pointcloud_downsampling | Downsampled pointcloud |
| `/scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan | 2D laser scan |
| `/simulated_pointcloud` | `sensor_msgs/PointCloud2` | lidar_simulator | Simulated pointcloud |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | Occupancy grid map |
| `/map_metadata` | `nav_msgs/MapMetaData` | slam_toolbox | Map metadata |

### Subscribed Topics
| Topic | Type | Package | Description |
|-------|------|---------|-------------|
| `/ouster/points` | `sensor_msgs/PointCloud2` | pointcloud_downsampling | Input dense pointcloud |
| `/downsampled_points` | `sensor_msgs/PointCloud2` | pointcloud_to_laserscan | Input for conversion |
| `/scan` | `sensor_msgs/LaserScan` | slam_toolbox | Input for SLAM |

---

## Troubleshooting

### QoS Incompatibility Warning
```
[WARN] requesting incompatible QoS. No messages will be sent to it.
```
**Solution:** The publisher uses RELIABLE QoS. Ensure subscribers match:
```bash
ros2 topic echo /scan --qos-reliability reliable
```

### No Laser Scan Output
1. Check TF tree: `ros2 run tf2_tools view_frames`
2. Verify height parameters include your pointcloud's Z range
3. Check topic connections: `ros2 topic info /scan`

### Build Errors with tf2_sensor_msgs
Ensure you're using `ament_target_dependencies()` in CMakeLists.txt:
```cmake
ament_target_dependencies(your_target
  tf2_sensor_msgs
  sensor_msgs
)
```

### SLAM Toolbox: "Message Filter dropping message"
This usually means TF is not properly configured:
1. Ensure `--clock` flag is used when playing rosbag
2. Verify all nodes have `use_sim_time: true`
3. Check TF chain exists: `ros2 run tf2_ros tf2_echo map os_sensor`

### SLAM Toolbox: "TF_OLD_DATA"
Time synchronization issue between bag and nodes:
1. Start rosbag FIRST before launching nodes
2. Use slower playback rate: `--rate 0.3`
3. Ensure static TF publishers have `use_sim_time: true`

### SLAM Map Not Updating
1. Check `minimum_travel_distance` and `minimum_travel_heading` in YAML
2. Set both to `0.0` to update on every scan
3. Verify `/scan` topic is publishing: `ros2 topic hz /scan`

---

## License

BSD-3-Clause

## Authors

- Original pointcloud_to_laserscan: Paul Bovbel (Willow Garage)
- Modifications