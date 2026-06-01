# FAST-Calib

This repository contains two ROS 2 packages for LiDAR-camera extrinsic calibration.

> [!WARNING]
> **This is an experimental ROS 2 port.** Not all functionality has been tested
> end-to-end. The core calibration pipeline (`FAST-Calib`) has been built and
> verified to compile. The interactive data collection package
> (`FAST-Calib-Interactive`) and the `distance_filter_tool.py` ROS 2 port are
> new and have not been validated against real hardware. Use with caution and
> expect rough edges.

---

## Packages

### [`FAST-Calib/`](FAST-Calib/)

The core calibration engine. Processes a pre-recorded ROS 2 bag and a camera
image to compute the LiDAR-camera extrinsic transformation matrix.

- **Type**: `ament_cmake` (C++17)
- **Executables**: `fast_calib` (single-scene), `multi_fast_calib` (multi-scene)
- **See**: [`FAST-Calib/README.md`](FAST-Calib/README.md) and [`FAST-Calib/USAGE.md`](FAST-Calib/USAGE.md)

### [`FAST-Calib-Interactive/`](FAST-Calib-Interactive/)

An interactive CLI front end for data collection and calibration orchestration.
Subscribes to live LiDAR and camera topics, records scenes to disk, and invokes
the `FAST-Calib` executables. Does not reimplement any calibration logic.

- **Type**: `ament_python`
- **Entry point**: `ros2 run fast_calib_interactive interactive_calib`
- **See**: [`FAST-Calib-Interactive/README.md`](FAST-Calib-Interactive/README.md)

---

## Build

These packages build as standard ROS 2 ament packages.

## Quick Start

**Offline (bag + image already collected):**
```bash
# Edit FAST-Calib/config/qr_params.yaml with your paths and intrinsics
ros2 launch fast_calib calib_launch.py
```

**Interactive (collect from live sensors):**
```bash
ros2 run fast_calib_interactive interactive_calib
```

---

## Prerequisites

- ROS 2 Jazzy or later
- PCL ≥ 1.10
- OpenCV ≥ 4.0 (with ArUco module)
- Python 3: `rosbag2_py`, `sensor_msgs_py`, `open3d`, `numpy`, `cv_bridge`

---

## Upstream

Based on [FAST-Calib](https://github.com/xuankuzcr/FAST-Calib) by Chunran Zheng (HKU MARS Lab). See the related paper:
[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210).
