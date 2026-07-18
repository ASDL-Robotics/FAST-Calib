#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utility for determining distance filter parameters for FAST-Calib (ROS 2).

Workflow:
1) Auto-detect the LiDAR point cloud message type in a rosbag2 directory:
   - sensor_msgs/msg/PointCloud2       (e.g. /hesai/pandar, /ouster/points)
   - livox_ros_driver2/msg/CustomMsg   (e.g. /livox/lidar)
2) Export all points to a PCD file with intensity (x y z intensity, ASCII).
3) Use Open3D to interactively pick at least 4 points, compute the bounding
   range from those points, and save the result to a .txt file with the same
   base name as the PCD.

Dependencies:
    - rosbag2_py
    - sensor_msgs_py.point_cloud2
    - rclpy.serialization
    - open3d, numpy

Usage:
    python3 distance_filter_tool.py <bag_dir> [output_dir]
    python3 distance_filter_tool.py /path/to/rosbag2_dir /path/to/output_dir

Note:
    In ROS 2 a "bag" is a directory containing a metadata.yaml file and one or
    more storage files (.db3 for sqlite3, .mcap for mcap). Pass the directory.
"""

import os
import sys

import numpy as np
import open3d as o3d

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

# Fully-qualified ROS 2 message type names.
POINTCLOUD2_TYPE = "sensor_msgs/msg/PointCloud2"
LIVOX_CUSTOM_TYPE = "livox_ros_driver2/msg/CustomMsg"

# ===================== Common: rosbag2 helpers =====================


def _detect_storage_id(bag_dir):
    """Infer the rosbag2 storage backend from the files in the bag directory."""
    for entry in os.listdir(bag_dir):
        if entry.endswith(".mcap"):
            return "mcap"
        if entry.endswith(".db3"):
            return "sqlite3"
    # Default to mcap (Jazzy and later).
    return "mcap"


def _open_reader(bag_dir):
    """Open a rosbag2 SequentialReader on the given bag directory."""
    storage_id = _detect_storage_id(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    return reader


def _topic_type_map(reader):
    """Return a {topic_name: type_string} map for the open reader."""
    return {t.name: t.type for t in reader.get_all_topics_and_types()}

# ===================== Common: Save PCD =====================


def save_pcd_with_intensity(points, intensities, output_path):
    """
    Save a point cloud as a PCD file with an intensity field (ASCII format).
    points: list/ndarray of [x, y, z]
    intensities: list/ndarray of intensity values
    """
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, 'w') as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] Saved point cloud with intensity field to: {output_path}")

# ===================== Case 1: PointCloud2 =====================


def find_intensity_field(msg):
    """Auto-detect the intensity field name in a PointCloud2 message."""
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_dir,
    output_dir,
    topic_name="/hesai/pandar",  # Change to match your topic name
    pcd_name="sensor_PointCloud2_inten_ascii.pcd"
):
    """
    Read all PointCloud2 messages from a rosbag2 topic and export them
    as a single merged PCD file. Coordinates are kept in the original
    LiDAR frame without any transformation.
    """
    print(f"[Bag] Opening rosbag2: {bag_dir}")
    reader = _open_reader(bag_dir)
    type_map = _topic_type_map(reader)

    if topic_name not in type_map:
        print(f"[ERROR] Topic '{topic_name}' not found in bag.", file=sys.stderr)
        print(f"[ERROR] Available topics: {list(type_map)}", file=sys.stderr)
        return None

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))

    all_points = []
    all_intensities = []
    intensity_field = None

    print(f"[Bag] Reading PointCloud2 messages from topic '{topic_name}'...")

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, PointCloud2)

        if intensity_field is None:
            intensity_field = find_intensity_field(msg)
            if intensity_field:
                print(f"[Bag] Detected intensity field: {intensity_field}")
            else:
                print("[ERROR] No intensity field found. Aborting conversion.",
                      file=sys.stderr)
                return None

        try:
            field_names = ["x", "y", "z", intensity_field]
            for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                all_points.append([point[0], point[1], point[2]])
                all_intensities.append(point[3])  # intensity is the fourth field
        except Exception as e:
            print(f"[ERROR] Read error: {str(e)}", file=sys.stderr)
            continue

    if not all_points:
        print("[ERROR] No PointCloud2 data found.", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    intensities = np.array(all_intensities, dtype=np.float32)
    save_pcd_with_intensity(all_points, intensities, output_path)
    return output_path

# ===================== Case 2: Livox CustomMsg =====================


def parse_livox_custom_msg(msg):
    """
    Parse x, y, z, and reflectivity from a livox_ros_driver2/CustomMsg.
    msg.points is a list of CustomPoint objects with fields
    x, y, z, and reflectivity.
    """
    points = []
    intensities = []

    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)

    return points, intensities


def convert_livox_custom_bag_to_pcd(
    bag_dir,
    output_dir,
    topic_name="/livox/lidar",  # Change to match your topic name
    pcd_name="livox_CustomMsg_inten_ascii.pcd"
):
    """
    Read all livox_ros_driver2/CustomMsg messages from a rosbag2 topic and
    export them as a single merged PCD file. Coordinates are kept in the
    original LiDAR frame without any transformation.
    """
    print(f"[Bag] Opening rosbag2: {bag_dir}")
    reader = _open_reader(bag_dir)
    type_map = _topic_type_map(reader)

    if topic_name not in type_map:
        print(f"[ERROR] Topic '{topic_name}' not found in bag.", file=sys.stderr)
        print(f"[ERROR] Available topics: {list(type_map)}", file=sys.stderr)
        return None

    # Resolve the CustomMsg class dynamically; it lives in an external package
    # that may not be installed. Fail with a clear message if missing.
    try:
        custom_msg_cls = get_message(type_map[topic_name])
    except (ModuleNotFoundError, ValueError) as e:
        print(f"[ERROR] Cannot load message type '{type_map[topic_name]}': {e}",
              file=sys.stderr)
        print("[ERROR] Ensure livox_ros_driver2 is built and sourced.",
              file=sys.stderr)
        return None

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))

    all_points = []
    all_intensities = []

    print(f"[Bag] Reading CustomMsg messages from topic '{topic_name}'...")

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, custom_msg_cls)
        pts, intens = parse_livox_custom_msg(msg)
        all_points.extend(pts)
        all_intensities.extend(intens)

    if not all_points:
        print("[ERROR] No Livox CustomMsg data found.", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    intensities = np.array(all_intensities, dtype=np.float32)
    save_pcd_with_intensity(all_points, intensities, output_path)
    return output_path

# ===================== Auto-detect: Determine message type =====================


def detect_lidar_msg_type(bag_dir):
    """
    Scan the bag's topic metadata for PointCloud2 or Livox CustomMsg messages.
    Returns a tuple (kind, topic_name):
        ("PointCloud2", topic), ("CustomMsg", topic), or (None, None)
    If both types are present, PointCloud2 takes priority.
    """
    print(f"[Detect] Scanning bag: {bag_dir}")
    reader = _open_reader(bag_dir)
    type_map = _topic_type_map(reader)

    pc2_topic = None
    livox_topic = None
    for name, type_str in type_map.items():
        if type_str == POINTCLOUD2_TYPE and pc2_topic is None:
            pc2_topic = name
        elif type_str == LIVOX_CUSTOM_TYPE and livox_topic is None:
            livox_topic = name

    if pc2_topic and livox_topic:
        print("[Detect] Both PointCloud2 and Livox CustomMsg detected. "
              "Defaulting to PointCloud2.")
        return "PointCloud2", pc2_topic
    elif pc2_topic:
        print(f"[Detect] Detected PointCloud2 messages on '{pc2_topic}'.")
        return "PointCloud2", pc2_topic
    elif livox_topic:
        print(f"[Detect] Detected Livox CustomMsg messages on '{livox_topic}'.")
        return "CustomMsg", livox_topic
    else:
        print("[Detect] No supported LiDAR message type found.")
        return None, None

# ============ Open3D interactive point picking & range saving ============


def select_and_save_points(pcd_folder, target_pcd_name):
    """
    Load a PCD file from the given directory, interactively pick points
    using Open3D, and save the bounding range to a .txt file.
    """
    pcd_path = os.path.join(pcd_folder, target_pcd_name)
    if not os.path.isfile(pcd_path):
        print(f"[ERROR] PCD file not found: {pcd_path}", file=sys.stderr)
        return

    # Load point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)
    if not pcd.has_points():
        print(f"[ERROR] {target_pcd_name} contains no points, skipping.", file=sys.stderr)
        return

    print(f"\nProcessing: {target_pcd_name}")
    print("In the visualization window, hold Shift and left-click to select points "
          "(at least 4), then press Q to close.")

    # Open interactive visualization window
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name=f"Select Points - {target_pcd_name}")
    vis.add_geometry(pcd)

    # Wait for user interaction (Shift+left-click to pick, Q to quit)
    vis.run()
    vis.destroy_window()

    # Retrieve indices of user-selected points
    selected_indices = vis.get_picked_points()

    if not selected_indices:
        print(f"[ERROR] No points selected. No file saved for {target_pcd_name}.",
              file=sys.stderr)
        return

    if len(selected_indices) < 4:
        print(f"[ERROR] Only {len(selected_indices)} point(s) selected; "
              f"at least 4 required. Skipping.", file=sys.stderr)
        return

    # Use only the first 4 selected points
    selected_indices = selected_indices[:4]

    all_points = np.asarray(pcd.points)
    selected_points = all_points[selected_indices, :]  # shape (4, 3)

    # Compute per-axis min/max of the 4 selected points
    mins = selected_points.min(axis=0)  # [x_min_raw, y_min_raw, z_min_raw]
    maxs = selected_points.max(axis=0)  # [x_max_raw, y_max_raw, z_max_raw]

    # Expand bounds by 0.2 m on each side
    x_min = mins[0] - 0.2
    x_max = maxs[0] + 0.2
    y_min = mins[1] - 0.2
    y_max = maxs[1] + 0.2
    z_min = mins[2] - 0.2
    z_max = maxs[2] + 0.2

    # Save to a .txt file with the same base name as the PCD
    base_name = os.path.splitext(target_pcd_name)[0]
    save_file = os.path.join(pcd_folder, f"{base_name}.txt")

    with open(save_file, 'w') as f:
        f.write("# 4 selected points (x y z)\n")
        for p in selected_points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        f.write("# range values in order:\n")
        f.write(f"x_min: {x_min:.1f}\n")
        f.write(f"x_max: {x_max:.1f}\n")
        f.write(f"y_min: {y_min:.1f}\n")
        f.write(f"y_max: {y_max:.1f}\n")
        f.write(f"z_min: {z_min:.1f}\n")
        f.write(f"z_max: {z_max:.1f}\n")

    print(f"[Save] Saved selected points and range to: {save_file}")
    print("Done.")

# ===================== main =====================


def main():
    # Parse command-line arguments: bag directory and output directory
    if len(sys.argv) > 1:
        bag_dir = sys.argv[1]
    else:
        print("Usage: python3 distance_filter_tool.py <bag_dir> [output_dir]",
              file=sys.stderr)
        sys.exit(1)

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"No output directory specified. Using current directory: {output_dir}")

    if not os.path.isdir(bag_dir):
        print(f"[ERROR] Bag directory '{bag_dir}' does not exist "
              f"(ROS 2 bags are directories).", file=sys.stderr)
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] Output directory '{output_dir}' does not exist.", file=sys.stderr)
        sys.exit(1)

    # Auto-detect the LiDAR message type in the bag
    msg_type, topic_name = detect_lidar_msg_type(bag_dir)
    if msg_type is None:
        print("[ERROR] No supported LiDAR message type detected. Exiting.",
              file=sys.stderr)
        sys.exit(1)

    # Convert to PCD based on detected type, using the auto-detected topic
    if msg_type == "PointCloud2":
        pcd_path = convert_pointcloud2_bag_to_pcd(
            bag_dir=bag_dir,
            output_dir=output_dir,
            topic_name=topic_name,
            pcd_name="sensor_PointCloud2_inten_ascii.pcd"
        )
    else:  # "CustomMsg"
        pcd_path = convert_livox_custom_bag_to_pcd(
            bag_dir=bag_dir,
            output_dir=output_dir,
            topic_name=topic_name,
            pcd_name="livox_CustomMsg_inten_ascii.pcd"
        )

    if pcd_path is None:
        print("[ERROR] PCD generation failed. Exiting.", file=sys.stderr)
        sys.exit(1)

    # Interactively pick points and save the filter range
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )


if __name__ == "__main__":
    main()
