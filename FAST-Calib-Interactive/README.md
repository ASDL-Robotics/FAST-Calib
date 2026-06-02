# FAST-Calib-Interactive

An interactive CLI front end for [FAST-Calib](../FAST-Calib). It handles the parts
FAST-Calib intentionally leaves out: **collecting data from live sensors** and
**orchestrating calibration runs**. The calibration math itself is untouched —
this package only records scenes and invokes the existing `fast_calib` and
`multi_fast_calib` executables.

## What it does

1. Subscribes to your live LiDAR (`PointCloud2`) and camera (`Image`) topics.
2. Records each scene to a rosbag2 directory plus a snapshot image.
3. Writes the scene paths into a per-scene copy of `qr_params.yaml`.
4. Calls `fast_calib` for single-scene calibration and `multi_fast_calib` for
   multi-scene refinement.

The original `fast_calib` package is never modified.

## Build

```bash
cd ~/Workspaces/fastcal_ws
colcon build --packages-select fast_calib fast_calib_interactive
source install/setup.bash
```

## Run

```bash
ros2 run fast_calib_interactive interactive_calib
```

Options:

```bash
ros2 run fast_calib_interactive interactive_calib \
  --config /path/to/qr_params.yaml \
  --output ~/calib_runs
```

If `--config` is omitted, the tool uses the `qr_params.yaml` installed with the
`fast_calib` package. If `--output` is omitted, it writes to
`./fast_calib_interactive_output`.

## Menu

```
1. Collect new scene          # record bag + image from live topics
2. List collected scenes
3. Run single-scene calibration (last scene)
4. Run multi-scene calibration   # needs >= 3 scenes
5. Exit
```

## Output layout

```
<output>/
├── scenes/
│   ├── scene1/
│   │   ├── lidar_bag/        # rosbag2 (sqlite3)
│   │   └── image.png
│   └── ...
├── configs/                  # generated per-scene param files
├── calib_result.txt          # from fast_calib
├── multi_calib_result.txt    # from multi_fast_calib
└── colored_cloud.pcd
```

## Notes

- Camera intrinsics still come from `qr_params.yaml`. Set them before running.
- The distance filter (`x_min`/`x_max`/...) also comes from `qr_params.yaml`.
  Use `FAST-Calib/scripts/distance_filter_tool.py` to determine good bounds.
- Multi-scene calibration reads `circle_center_record.txt`, which each
  single-scene run appends to. Run single-scene on 3+ scenes first.
