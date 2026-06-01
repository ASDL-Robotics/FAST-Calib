# FAST-Calib-Interactive

An interactive CLI front end for [FAST-Calib](../FAST-Calib). It handles **live data collection from multiple sensors** and **orchestrating calibration runs**. All calibration math lives in the `fast_calib` C++ package — this package only collects data and invokes the existing executables.

## What it does

1. Reads a `sensors.yaml` that declares your cameras and LiDARs (created interactively on first run if not found).
2. Groups cameras by their associated LiDAR. For each LiDAR group, records **one shared bag** while simultaneously capturing images and intrinsics from all paired cameras — no redundant LiDAR data.
3. Camera intrinsics are fetched live from each camera's `CameraInfo` topic. No manual entry required.
4. Generates a `qr_params.yaml` per scene per camera from the live intrinsics and target geometry, written to `$XDG_STATE_HOME`.
5. Calls `fast_calib` (single-scene) and `multi_fast_calib` (multi-scene) for each camera-LiDAR pair.
6. Derives inter-camera transforms automatically for cameras sharing a LiDAR.

## Run

```bash
ros2 run fast_calib_interactive interactive_calib
```

On first run, if no `sensors.yaml` is found, the wizard walks you through creating one. Subsequent runs load it automatically.

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--sensors` | `$XDG_CONFIG_HOME/ros/fast_calib/sensors.yaml` | Sensor suite config. Created interactively if missing. |
| `--output` | `$XDG_STATE_HOME/ros/fast_calib/result` | Directory for final calibration results. |
| `--state` | `$XDG_STATE_HOME/ros/fast_calib` | Directory for transient working data (bags, images, generated configs). |

## First-run wizard

If `sensors.yaml` is not found at the default or specified path, the wizard prompts you to define your sensor suite:

```
========================================
  sensors.yaml not found.
  Let's create one now.
========================================

--- LiDARs ---
  name [lidar_1]: lidar_main
  topic [/livox/lidar]: /livox/lidar

--- Cameras ---
  name [cam_1]: cam_front
  image_topic [/camera/cam_front/image_raw]: /camera/front/image_raw
  info_topic  [/camera/cam_front/camera_info]: /camera/front/camera_info
  lidar: lidar_main

--- Calibration Target ---
  marker_size [0.16]:
  ...

  sensors.yaml written to ~/.config/ros/fast_calib/sensors.yaml
```

The file is saved to `$XDG_CONFIG_HOME/ros/fast_calib/sensors.yaml` and reused on all future runs. Edit it directly to change your sensor configuration.

## sensors.yaml format

```yaml
sensors:
  cameras:
    - name: cam_front
      image_topic: /camera/front/image_raw
      info_topic:  /camera/front/camera_info   # intrinsics read from here at runtime
      lidar: lidar_main                         # must match a lidar name below

    - name: cam_left
      image_topic: /camera/left/image_raw
      info_topic:  /camera/left/camera_info
      lidar: lidar_main

  lidars:
    - name: lidar_main
      topic: /livox/lidar                       # e.g. /ouster/points, /velodyne_points

# Calibration target geometry — measure your physical target.
target:
  marker_size: 0.16
  delta_width_qr_center: 0.47
  delta_height_qr_center: 0.27
  delta_width_circles: 0.4
  delta_height_circles: 0.3
  circle_radius: 0.10
  min_detected_markers: 3
```

Camera intrinsics are **not** specified here — they are fetched live from each camera's `CameraInfo` topic during collection.

## Menu

```
  1. Collect new scene (all cameras)
  2. List collected scenes
  3. Run single-scene calibration (last scene)
  4. Run multi-scene calibration
  5. Compute inter-camera transforms
  6. Exit
```

**Recommended workflow:**

1. Collect 3+ scenes with the calibration target in different orientations.
2. Run single-scene calibration after each scene to verify detection.
3. Run multi-scene calibration for the best accuracy.
4. Compute inter-camera transforms (requires ≥ 2 cameras sharing a LiDAR).

## Directory layout

```
$XDG_CONFIG_HOME/ros/fast_calib/
└── sensors.yaml                            ← user config (hand-editable)

$XDG_STATE_HOME/ros/fast_calib/
├── scenes/
│   ├── scene1/
│   │   ├── lidar_main/
│   │   │   └── lidar_bag/                  ← one bag shared by all cameras on this LiDAR
│   │   ├── cam_front/
│   │   │   └── image.png
│   │   └── cam_left/
│   │       └── image.png
│   └── scene2/ ...
├── configs/
│   ├── scene1_cam_front_params.yaml        ← generated qr_params (transient)
│   ├── scene1_cam_left_params.yaml
│   └── multi_cam_front_params.yaml
└── result/                                 ← default output (override with --output)
    ├── cam_front/
    │   ├── calib_result.txt
    │   ├── multi_calib_result.txt
    │   ├── colored_cloud.pcd
    │   └── qr_detect.png
    ├── cam_left/
    │   └── ...
    └── inter_camera/
        └── T_cam_left_cam_front.txt        ← derived inter-camera transform
```

## Multi-camera calibration

Cameras sharing a LiDAR are recorded simultaneously in a single collection
pass — the LiDAR bag is written once and referenced by all paired cameras.

After calibration, the inter-camera transform is derived as:

```
T_camB_camA = T_camB_lidar × inv(T_camA_lidar)
```

This is computed automatically by menu option 5 once both cameras have been
calibrated. Results are saved to `<output>/inter_camera/`.

## Notes

- The distance filter (`x_min`/`x_max`/...) defaults to a wide passthrough.
  Use `FAST-Calib/scripts/distance_filter_tool.py` to determine tighter bounds,
  then add them to the generated `qr_params.yaml` in `$XDG_STATE_HOME` or pass
  them as ROS 2 parameter overrides.
- `circle_center_record.txt` accumulates in `result/<camera>/` across
  single-scene runs. Multi-scene calibration reads it from there.
- Each single-scene run overwrites `calib_result.txt` for that camera.
  Multi-scene writes `multi_calib_result.txt` separately.
