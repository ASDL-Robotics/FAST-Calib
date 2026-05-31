# FAST-Calib Usage Guide

## Overview

FAST-Calib is an offline calibration tool that processes pre-recorded ROS 2 bag files and camera images to compute LiDAR-camera extrinsic calibration parameters.

---

## Quick Start

### 1. Prepare Your Data

Collect calibration data for at least 3 different scenes:

```bash
# For each scene:
# 1. Position calibration target in view of both sensors
# 2. Record LiDAR data
ros2 bag record /livox/lidar -o scene1

# 3. Capture synchronized camera image
ros2 run image_view image_saver --ros-args -r image:=/camera/image_raw
# Or use your preferred method to save an image
```

### 2. Configure Parameters

Edit the configuration file:

```bash
nano config/qr_params.yaml
```

Update the file paths and camera intrinsics (see Configuration section below).

### 3. Run Single-Scene Calibration

For each scene:

```bash
# Update config/qr_params.yaml with scene-specific paths
ros2 launch fast_calib calib_launch.py
```

### 4. Run Multi-Scene Refinement

After processing 3+ scenes:

```bash
ros2 launch fast_calib multi_calib.launch.py
```

---

## File Locations

### Input Files (You Provide)

| File Type | Description | Example Path |
|-----------|-------------|--------------|
| **ROS 2 Bag** | Recorded LiDAR point cloud data | `/path/to/scene1/` |
| **Camera Image** | Synchronized camera frame (PNG, JPG, etc.) | `/path/to/scene1.png` |
| **Calibration Target** | Physical ArUco board with circular holes | See README.md for CAD model |

### Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| **qr_params.yaml** | `config/qr_params.yaml` | Main configuration file |
| **calib_launch.py** | `launch/calib_launch.py` | Single-scene launch file |
| **multi_calib.launch.py** | `launch/multi_calib.launch.py` | Multi-scene launch file |
| **fast_livo2.rviz** | `rviz_cfg/fast_livo2.rviz` | RViz visualization config |

### Output Files (Generated)

| File | Location | Description |
|------|----------|-------------|
| **calib_result.txt** | `<output_path>/calib_result.txt` | Single-scene calibration result (FAST-LIVO2 format) |
| **multi_calib_result.txt** | `<output_path>/multi_calib_result.txt` | Multi-scene refined calibration |
| **colored_cloud.pcd** | `<output_path>/colored_cloud.pcd` | LiDAR point cloud colored by camera |
| **qr_detect.png** | `<output_path>/qr_detect.png` | Image with detected markers/circles |
| **circle_center_record.txt** | `<output_path>/circle_center_record.txt` | Accumulated circle centers from all scenes |

---

## Configuration Parameters

### Required Parameters (Must Configure)

Edit `config/qr_params.yaml`:

```yaml
fast_calib:
  ros__parameters:
    # ===== CAMERA INTRINSICS (from camera calibration) =====
    fx: 617.123          # Focal length X (pixels)
    fy: 617.456          # Focal length Y (pixels)
    cx: 320.5            # Principal point X (pixels)
    cy: 240.5            # Principal point Y (pixels)
    k1: -0.123           # Radial distortion k1
    k2: 0.045            # Radial distortion k2
    p1: 0.001            # Tangential distortion p1
    p2: -0.002           # Tangential distortion p2

    # ===== INPUT FILES =====
    bag_path: "/home/user/calib_data/scene1"
    image_path: "/home/user/calib_data/scene1.png"
    lidar_topic: "/livox/lidar"    # Topic name in the bag file
    
    # ===== OUTPUT DIRECTORY =====
    output_path: "/home/user/calib_output"

    # ===== DISTANCE FILTER (adjust to frame the target) =====
    x_min: 1.5           # Minimum X distance (meters)
    x_max: 3.0           # Maximum X distance (meters)
    y_min: -1.5          # Minimum Y distance (meters)
    y_max: 2.0           # Maximum Y distance (meters)
    z_min: -0.5          # Minimum Z distance (meters)
    z_max: 2.0           # Maximum Z distance (meters)
```

### Calibration Target Parameters (Default Values)

These match the provided CAD model. Only change if using a custom target:

```yaml
    # ===== TARGET GEOMETRY =====
    marker_size: 0.16              # ArUco marker size (meters)
    delta_width_qr_center: 0.47    # Half horizontal distance between marker centers
    delta_height_qr_center: 0.27   # Half vertical distance between marker centers
    delta_width_circles: 0.4       # Horizontal distance between circle centers
    delta_height_circles: 0.3      # Vertical distance between circle centers
    circle_radius: 0.10            # Radius of circular holes (meters)
    min_detected_markers: 3        # Minimum ArUco markers required
```

---

## Command-Line Usage

### Method 1: Using Launch Files (Recommended)

```bash
# Single-scene calibration with RViz
ros2 launch fast_calib calib_launch.py

# Single-scene without RViz
ros2 launch fast_calib calib_launch.py rviz:=false

# Multi-scene calibration
ros2 launch fast_calib multi_calib.launch.py
```

### Method 2: Direct Node Execution

```bash
# Single-scene calibration
ros2 run fast_calib fast_calib --ros-args --params-file config/qr_params.yaml

# Multi-scene calibration
ros2 run fast_calib multi_fast_calib --ros-args --params-file config/qr_params.yaml
```

### Method 3: Command-Line Parameter Override

Override specific parameters without editing the YAML file:

```bash
ros2 run fast_calib fast_calib \
  --ros-args \
  --params-file config/qr_params.yaml \
  -p bag_path:=/home/user/data/scene2 \
  -p image_path:=/home/user/data/scene2.png \
  -p output_path:=/home/user/results
```

---

## Parameter Reference

### Input/Output Parameters

| Parameter | Type | Description | Example |
|-----------|------|-------------|---------|
| `bag_path` | string | Path to ROS 2 bag directory | `/home/user/scene1` |
| `image_path` | string | Path to camera image file | `/home/user/scene1.png` |
| `lidar_topic` | string | LiDAR topic name in bag | `/livox/lidar`, `/ouster/points` |
| `output_path` | string | Directory for output files | `/home/user/output` |

### Camera Intrinsics

| Parameter | Type | Description | Typical Range |
|-----------|------|-------------|---------------|
| `fx` | double | Focal length X (pixels) | 400-800 |
| `fy` | double | Focal length Y (pixels) | 400-800 |
| `cx` | double | Principal point X (pixels) | Image width / 2 |
| `cy` | double | Principal point Y (pixels) | Image height / 2 |
| `k1` | double | Radial distortion coefficient 1 | -0.5 to 0.5 |
| `k2` | double | Radial distortion coefficient 2 | -0.5 to 0.5 |
| `p1` | double | Tangential distortion coefficient 1 | -0.01 to 0.01 |
| `p2` | double | Tangential distortion coefficient 2 | -0.01 to 0.01 |

**Note**: Obtain these from camera calibration (e.g., using ROS 2 `camera_calibration` package or OpenCV).

### Distance Filter (ROI)

| Parameter | Type | Description | Units |
|-----------|------|-------------|-------|
| `x_min` | double | Minimum X coordinate | meters |
| `x_max` | double | Maximum X coordinate | meters |
| `y_min` | double | Minimum Y coordinate | meters |
| `y_max` | double | Maximum Y coordinate | meters |
| `z_min` | double | Minimum Z coordinate | meters |
| `z_max` | double | Maximum Z coordinate | meters |

**Tip**: Use `scripts/distance_filter_tool.py` to visualize and determine optimal filter bounds.

### Target Geometry

| Parameter | Type | Description | Default | Units |
|-----------|------|-------------|---------|-------|
| `marker_size` | double | ArUco marker edge length | 0.16 | meters |
| `delta_width_qr_center` | double | Half horizontal spacing of markers | 0.47 | meters |
| `delta_height_qr_center` | double | Half vertical spacing of markers | 0.27 | meters |
| `delta_width_circles` | double | Horizontal spacing of circles | 0.4 | meters |
| `delta_height_circles` | double | Vertical spacing of circles | 0.3 | meters |
| `circle_radius` | double | Radius of circular holes | 0.10 | meters |
| `min_detected_markers` | int | Minimum ArUco markers to detect | 3 | count |

---

## Workflow Examples

### Example 1: Basic Single-Scene Calibration

```bash
# 1. Edit configuration
nano config/qr_params.yaml
# Set:
#   bag_path: /home/user/calib/scene1
#   image_path: /home/user/calib/scene1.png
#   output_path: /home/user/calib/output

# 2. Run calibration
ros2 launch fast_calib calib_launch.py

# 3. Check results
cat /home/user/calib/output/calib_result.txt
```

### Example 2: Multi-Scene Calibration

```bash
# Scene 1 (target facing forward)
nano config/qr_params.yaml  # Set scene1 paths
ros2 launch fast_calib calib_launch.py rviz:=false

# Scene 2 (target oriented right)
nano config/qr_params.yaml  # Set scene2 paths
ros2 launch fast_calib calib_launch.py rviz:=false

# Scene 3 (target oriented left)
nano config/qr_params.yaml  # Set scene3 paths
ros2 launch fast_calib calib_launch.py rviz:=false

# Multi-scene refinement
ros2 launch fast_calib multi_calib.launch.py

# Check refined results
cat /home/user/calib/output/multi_calib_result.txt
```

### Example 3: Using Different LiDAR Types

```yaml
# For Livox Mid360
lidar_topic: "/livox/lidar"

# For Ouster OS1/OS2
lidar_topic: "/ouster/points"

# For Velodyne
lidar_topic: "/velodyne_points"
```

The system automatically detects solid-state vs. mechanical LiDAR based on the presence of a `ring` field.

---

## Output Format

### Single-Scene Result (`calib_result.txt`)

```
# FAST-LIVO2 calibration format
cam_model: Pinhole
cam_width: 640
cam_height: 480
scale: 1.0
cam_fx: 617.123000
cam_fy: 617.456000
cam_cx: 320.500000
cam_cy: 240.500000
cam_d0: -0.123000
cam_d1: 0.045000
cam_d2: 0.001000
cam_d3: -0.002000

Rcl: [ 0.999123, -0.041234,  0.005678,
       0.041567,  0.998765, -0.023456,
      -0.004321,  0.023789,  0.999712]
Pcl: [ 0.123456,  0.234567,  0.345678]
```

- **Rcl**: 3×3 rotation matrix (camera ← LiDAR)
- **Pcl**: 3×1 translation vector (camera ← LiDAR)

### Multi-Scene Result (`multi_calib_result.txt`)

```
# FAST-LIVO2 calibration format
Rcl: [ 0.999234, -0.039123,  0.004567,
       0.039456,  0.999123, -0.021234,
      -0.003456,  0.021567,  0.999765]
Pcl: [ 0.125678,  0.236789,  0.347890]
```

Same format but refined using all scenes.

---

## Troubleshooting

### Issue: "No points loaded from bag"

**Cause**: Incorrect topic name or bag format.

**Solution**:
```bash
# Check available topics in bag
ros2 bag info /path/to/bag

# Update lidar_topic in qr_params.yaml to match
```

### Issue: "Unable to find candidate set that matches target's geometry"

**Cause**: Distance filter too tight or target not fully visible.

**Solution**:
1. Use `scripts/distance_filter_tool.py` to visualize point cloud
2. Adjust `x_min`, `x_max`, `y_min`, `y_max`, `z_min`, `z_max` to include entire target
3. Ensure target is not occluded

### Issue: "Marker(s) found != 4"

**Cause**: ArUco markers not detected in image.

**Solution**:
1. Check image quality (lighting, focus, resolution)
2. Verify marker IDs are 1, 2, 3, 4
3. Reduce `min_detected_markers` to 3 if one marker is occluded
4. Check `marker_size` parameter matches physical marker size

### Issue: High RMSE (>0.05m)

**Cause**: Poor calibration quality.

**Solution**:
1. Verify camera intrinsics are correct
2. Ensure image and bag are temporally synchronized
3. Collect more diverse scenes (different angles/distances)
4. Run multi-scene calibration for refinement

---

## Advanced Usage

### Custom Calibration Target

If using a custom target design:

1. Measure physical dimensions accurately
2. Update target geometry parameters in `qr_params.yaml`
3. Ensure ArUco markers use DICT_6X6_250 dictionary with IDs 1, 2, 3, 4

### Batch Processing Multiple Scenes

Create a shell script:

```bash
#!/bin/bash
SCENES=("scene1" "scene2" "scene3")
BASE_PATH="/home/user/calib_data"
OUTPUT="/home/user/calib_output"

for scene in "${SCENES[@]}"; do
    echo "Processing $scene..."
    ros2 run fast_calib fast_calib \
        --ros-args \
        --params-file config/qr_params.yaml \
        -p bag_path:="$BASE_PATH/$scene" \
        -p image_path:="$BASE_PATH/$scene.png" \
        -p output_path:="$OUTPUT"
done

echo "Running multi-scene calibration..."
ros2 run fast_calib multi_fast_calib \
    --ros-args \
    --params-file config/qr_params.yaml \
    -p output_path:="$OUTPUT"
```

### Visualization Topics (DEBUG mode)

When `DEBUG=1` (default), the following topics are published:

| Topic | Type | Description |
|-------|------|-------------|
| `/qr_cloud` | PointCloud2 | Detected circle centers from camera |
| `/center_cloud` | PointCloud2 | Detected circle centers from LiDAR |
| `/filtered_cloud` | PointCloud2 | Distance-filtered point cloud |
| `/plane_cloud` | PointCloud2 | Plane-segmented points |
| `/aligned_cloud` | PointCloud2 | Plane-aligned points (Z=0) |
| `/edge_cloud` | PointCloud2 | Extracted edge points |
| `/center_z0_cloud` | PointCloud2 | Circle centers in aligned frame |
| `/aligned_lidar_centers` | PointCloud2 | Transformed LiDAR centers |
| `/colored_cloud` | PointCloud2 | Final colored point cloud |

View in RViz:
```bash
ros2 launch fast_calib calib_launch.py rviz:=true
```

---

## System Requirements

- **ROS 2**: Jazzy or later
- **PCL**: 1.10 or later
- **OpenCV**: 4.0 or later (with ArUco module)
- **Operating System**: Linux (tested on Ubuntu 22.04+)

