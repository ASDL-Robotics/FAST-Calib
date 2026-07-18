"""Replay a collected dataset on namespaced debug topics.

Reads a rosbag2 bag (mcap), applies the same X/Y/Z passthrough crop used by
``lidar_detect.hpp``, and publishes the filtered PointCloud2 messages on
``/fast_calib/debug/<dataset_name>/pointcloud``. If camera images exist for
the scene, they are published as well on
``/fast_calib/debug/<dataset_name>/<camera_name>/image``.

Additionally publishes:
- A wireframe box marker showing the filter bounds
- Sphere markers at detected cluster centroids (via Euclidean clustering
  on the filtered cloud, matching lidar_detect.hpp parameters)

The replay loops continuously until the configured duration expires so that
downstream consumers (e.g. RViz) have time to visualize the data regardless of
how short the original recording was.
"""

from __future__ import annotations

import math
import struct
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import rosbag2_py
from rclpy.serialization import deserialize_message

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except ImportError:  # pragma: no cover
    _HAS_CV = False

_DEBUG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

_TOPIC_PREFIX = '/fast_calib/debug'

# Euclidean clustering parameters matching lidar_detect.hpp
_CLUSTER_TOLERANCE = 0.05   # 5 cm
_MIN_CLUSTER_SIZE = 50
_MAX_CLUSTER_SIZE = 1000


# ---------------------------------------------------------------------------
# Point cloud filtering
# ---------------------------------------------------------------------------

def _find_xyz_offsets(fields: List[PointField]) -> Tuple[int, int, int]:
    """Return (x_offset, y_offset, z_offset) from PointCloud2 fields."""
    offsets = {}
    for f in fields:
        if f.name in ('x', 'y', 'z'):
            offsets[f.name] = f.offset
    return offsets['x'], offsets['y'], offsets['z']


def _extract_xyz(msg: PointCloud2) -> List[Tuple[float, float, float]]:
    """Extract (x, y, z) tuples from a PointCloud2 message."""
    x_off, y_off, z_off = _find_xyz_offsets(msg.fields)
    point_step = msg.point_step
    data = bytes(msg.data)
    num_points = msg.width * msg.height
    points = []
    for i in range(num_points):
        offset = i * point_step
        x = struct.unpack_from('<f', data, offset + x_off)[0]
        y = struct.unpack_from('<f', data, offset + y_off)[0]
        z = struct.unpack_from('<f', data, offset + z_off)[0]
        points.append((x, y, z))
    return points


def _filter_pointcloud(
    msg: PointCloud2,
    bounds: Dict[str, float],
) -> PointCloud2:
    """Apply an X/Y/Z passthrough filter to a PointCloud2 message.

    Mirrors the PCL PassThrough filter chain in lidar_detect.hpp.
    Operates on the raw byte buffer so no PCL dependency is needed.
    """
    x_off, y_off, z_off = _find_xyz_offsets(msg.fields)
    point_step = msg.point_step
    data = bytes(msg.data)
    num_points = msg.width * msg.height

    x_min = bounds.get('x_min', -float('inf'))
    x_max = bounds.get('x_max', float('inf'))
    y_min = bounds.get('y_min', -float('inf'))
    y_max = bounds.get('y_max', float('inf'))
    z_min = bounds.get('z_min', -float('inf'))
    z_max = bounds.get('z_max', float('inf'))

    kept = bytearray()
    kept_count = 0

    for i in range(num_points):
        offset = i * point_step
        x = struct.unpack_from('<f', data, offset + x_off)[0]
        y = struct.unpack_from('<f', data, offset + y_off)[0]
        z = struct.unpack_from('<f', data, offset + z_off)[0]

        if (x_min <= x <= x_max
                and y_min <= y <= y_max
                and z_min <= z <= z_max):
            kept.extend(data[offset:offset + point_step])
            kept_count += 1

    # Build a new PointCloud2 with the filtered points.
    filtered = PointCloud2()
    filtered.header = msg.header
    filtered.height = 1
    filtered.width = kept_count
    filtered.fields = msg.fields
    filtered.is_bigendian = msg.is_bigendian
    filtered.point_step = point_step
    filtered.row_step = kept_count * point_step
    filtered.data = kept
    filtered.is_dense = True
    return filtered


# ---------------------------------------------------------------------------
# Euclidean clustering (simple Python implementation)
# ---------------------------------------------------------------------------

def _euclidean_clusters(
    points: List[Tuple[float, float, float]],
    tolerance: float = _CLUSTER_TOLERANCE,
    min_size: int = _MIN_CLUSTER_SIZE,
    max_size: int = _MAX_CLUSTER_SIZE,
) -> List[List[int]]:
    """Simple grid-based Euclidean clustering.

    Groups points that are within ``tolerance`` of each other. Returns a list
    of clusters, where each cluster is a list of point indices.
    """
    if not points:
        return []

    # Voxel grid spatial index for neighbor lookup.
    cell_size = tolerance
    grid: Dict[Tuple[int, int, int], List[int]] = {}
    for idx, (x, y, z) in enumerate(points):
        key = (int(math.floor(x / cell_size)),
               int(math.floor(y / cell_size)),
               int(math.floor(z / cell_size)))
        grid.setdefault(key, []).append(idx)

    visited = [False] * len(points)
    clusters: List[List[int]] = []

    for seed_idx in range(len(points)):
        if visited[seed_idx]:
            continue
        visited[seed_idx] = True

        cluster = [seed_idx]
        queue = [seed_idx]

        while queue:
            current = queue.pop()
            cx, cy, cz = points[current]
            # Check neighboring cells (3x3x3 neighborhood).
            base_key = (int(math.floor(cx / cell_size)),
                        int(math.floor(cy / cell_size)),
                        int(math.floor(cz / cell_size)))

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        nkey = (base_key[0] + dx, base_key[1] + dy, base_key[2] + dz)
                        for neighbor_idx in grid.get(nkey, ()):
                            if visited[neighbor_idx]:
                                continue
                            nx, ny, nz = points[neighbor_idx]
                            dist_sq = ((cx - nx) ** 2 + (cy - ny) ** 2 + (cz - nz) ** 2)
                            if dist_sq <= tolerance * tolerance:
                                visited[neighbor_idx] = True
                                cluster.append(neighbor_idx)
                                queue.append(neighbor_idx)

            if len(cluster) > max_size:
                break

        if min_size <= len(cluster) <= max_size:
            clusters.append(cluster)

    return clusters


def _cluster_centroids(
    points: List[Tuple[float, float, float]],
    clusters: List[List[int]],
) -> List[Tuple[float, float, float]]:
    """Compute the centroid of each cluster."""
    centroids = []
    for cluster in clusters:
        n = len(cluster)
        sx = sum(points[i][0] for i in cluster)
        sy = sum(points[i][1] for i in cluster)
        sz = sum(points[i][2] for i in cluster)
        centroids.append((sx / n, sy / n, sz / n))
    return centroids


# ---------------------------------------------------------------------------
# Marker builders
# ---------------------------------------------------------------------------

def _make_filter_box_marker(
    bounds: Dict[str, float],
    frame_id: str,
    namespace: str,
) -> Marker:
    """Create a wireframe (LINE_LIST) marker showing the filter bounding box."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.01  # line width
    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
    marker.lifetime = Duration(sec=0, nanosec=0)  # persistent

    x0 = bounds.get('x_min', 0.0)
    x1 = bounds.get('x_max', 0.0)
    y0 = bounds.get('y_min', 0.0)
    y1 = bounds.get('y_max', 0.0)
    z0 = bounds.get('z_min', 0.0)
    z1 = bounds.get('z_max', 0.0)

    # 8 corners of the box
    corners = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    ]

    # 12 edges as pairs of corner indices
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
        (4, 5), (5, 6), (6, 7), (7, 4),  # top face
        (0, 4), (1, 5), (2, 6), (3, 7),  # vertical edges
    ]

    for a, b in edges:
        pa = Point(x=corners[a][0], y=corners[a][1], z=corners[a][2])
        pb = Point(x=corners[b][0], y=corners[b][1], z=corners[b][2])
        marker.points.append(pa)
        marker.points.append(pb)

    return marker


def _make_cluster_markers(
    centroids: List[Tuple[float, float, float]],
    frame_id: str,
    namespace: str,
) -> MarkerArray:
    """Create sphere markers at each cluster centroid."""
    marker_array = MarkerArray()

    for i, (cx, cy, cz) in enumerate(centroids):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=cx, y=cy, z=cz)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.9)
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker_array.markers.append(marker)

    return marker_array


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class DatasetPublisher(Node):
    """Publish a collected dataset's bag and images on debug topics.

    Parameters
    ----------
    dataset_name:
        Human-readable name for this dataset (used in topic namespace).
    bag_path:
        Path to the rosbag2 directory to replay.
    image_paths:
        Optional list of (camera_name, image_file_path) tuples.
        Each image is published once on a latched topic.
    filter_bounds:
        Dict with x_min/x_max/y_min/y_max/z_min/z_max to crop the point
        cloud (same bounds used by lidar_detect.hpp). If None, publishes
        unfiltered.
    duration_sec:
        How long to keep publishing (loops the bag). Default 30 s.
    rate_hz:
        Publishing rate for point cloud messages within the loop.
    """

    def __init__(
        self,
        dataset_name: str,
        bag_path: Path,
        image_paths: Optional[List[tuple[str, Path]]] = None,
        filter_bounds: Optional[Dict[str, float]] = None,
        duration_sec: float = 30.0,
        rate_hz: float = 10.0,
    ):
        super().__init__('fast_calib_dataset_publisher')
        self.dataset_name = dataset_name
        self.bag_path = Path(bag_path)
        self.image_paths = image_paths or []
        self.filter_bounds = filter_bounds
        self.duration_sec = duration_sec
        self.rate_hz = rate_hz

        self._namespace = _TOPIC_PREFIX

        # Create the PointCloud2 publisher.
        self._cloud_topic = f'{self._namespace}/pointcloud'
        self._cloud_pub = self.create_publisher(
            PointCloud2, self._cloud_topic, _DEBUG_QOS
        )

        # Marker publishers.
        self._box_marker_pub = self.create_publisher(
            Marker, f'{self._namespace}/filter_box', _DEBUG_QOS
        )
        self._cluster_marker_pub = self.create_publisher(
            MarkerArray, f'{self._namespace}/clusters', _DEBUG_QOS
        )

        # Pre-load point cloud messages from the bag.
        self._cloud_msgs: List[PointCloud2] = []
        self._load_bag()

        # Compute cluster centroids from the first filtered cloud.
        self._cluster_centroids: List[Tuple[float, float, float]] = []
        if self._cloud_msgs:
            self._compute_clusters(self._cloud_msgs[0])

        # Publish images (latched via TRANSIENT_LOCAL).
        self._publish_images()

        # Publish the filter box marker (once, latched).
        if self.filter_bounds:
            self._publish_filter_box()

        # Publish cluster markers (once, latched).
        if self._cluster_centroids:
            self._publish_cluster_markers()

        filter_info = 'none (raw cloud)'
        if self.filter_bounds:
            b = self.filter_bounds
            filter_info = (
                f'x=[{b.get("x_min", "")}, {b.get("x_max", "")}] '
                f'y=[{b.get("y_min", "")}, {b.get("y_max", "")}] '
                f'z=[{b.get("z_min", "")}, {b.get("z_max", "")}]'
            )

        self.get_logger().info(
            f'Dataset publisher ready:\n'
            f'  topic     : {self._cloud_topic}\n'
            f'  messages  : {len(self._cloud_msgs)} point clouds\n'
            f'  filter    : {filter_info}\n'
            f'  clusters  : {len(self._cluster_centroids)}\n'
            f'  images    : {len(self.image_paths)}\n'
            f'  duration  : {self.duration_sec}s @ {self.rate_hz} Hz'
        )

    # --- bag loading ------------------------------------------------------

    def _load_bag(self) -> None:
        """Deserialize all PointCloud2 messages from the bag into memory."""
        if not self.bag_path.is_dir():
            self.get_logger().error(f'Bag path does not exist: {self.bag_path}')
            return

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''),
        )

        # Find the PointCloud2 topic (there should be exactly one).
        topic_types = reader.get_all_topics_and_types()
        pc_topics = [
            t.name for t in topic_types
            if t.type == 'sensor_msgs/msg/PointCloud2'
        ]

        while reader.has_next():
            topic_name, data, _timestamp = reader.read_next()
            if topic_name in pc_topics:
                msg = deserialize_message(data, PointCloud2)
                if self.filter_bounds:
                    msg = _filter_pointcloud(msg, self.filter_bounds)
                self._cloud_msgs.append(msg)

    # --- clustering -------------------------------------------------------

    def _compute_clusters(self, msg: PointCloud2) -> None:
        """Run Euclidean clustering on a filtered cloud and store centroids."""
        points = _extract_xyz(msg)
        if not points:
            return

        self.get_logger().info(
            f'Running Euclidean clustering on {len(points)} points '
            f'(tolerance={_CLUSTER_TOLERANCE}m, '
            f'min_size={_MIN_CLUSTER_SIZE}, max_size={_MAX_CLUSTER_SIZE})...'
        )

        clusters = _euclidean_clusters(points)
        self._cluster_centroids = _cluster_centroids(points, clusters)

        self.get_logger().info(
            f'Found {len(clusters)} clusters, centroids: {len(self._cluster_centroids)}'
        )

    # --- marker publishing ------------------------------------------------

    def _publish_filter_box(self) -> None:
        """Publish the filter bounding box as a wireframe marker."""
        # Use the frame_id from the first cloud message, or default.
        frame_id = 'map'
        if self._cloud_msgs:
            frame_id = self._cloud_msgs[0].header.frame_id or 'map'

        marker = _make_filter_box_marker(
            self.filter_bounds, frame_id, f'{self._namespace}/filter_box'
        )
        marker.header.stamp = self.get_clock().now().to_msg()
        self._box_marker_pub.publish(marker)
        self.get_logger().info(f'Published filter box marker on {self._namespace}/filter_box')

    def _publish_cluster_markers(self) -> None:
        """Publish sphere markers at cluster centroids."""
        frame_id = 'map'
        if self._cloud_msgs:
            frame_id = self._cloud_msgs[0].header.frame_id or 'map'

        marker_array = _make_cluster_markers(
            self._cluster_centroids, frame_id, f'{self._namespace}/clusters'
        )
        # Stamp all markers.
        now = self.get_clock().now().to_msg()
        for m in marker_array.markers:
            m.header.stamp = now

        self._cluster_marker_pub.publish(marker_array)
        self.get_logger().info(
            f'Published {len(self._cluster_centroids)} cluster markers '
            f'on {self._namespace}/clusters'
        )

    # --- image publishing -------------------------------------------------

    def _publish_images(self) -> None:
        """Publish each camera image once on a latched topic."""
        if not _HAS_CV:
            if self.image_paths:
                self.get_logger().warn(
                    'cv_bridge/opencv not available — skipping image publishing.'
                )
            return

        bridge = CvBridge()
        for camera_name, img_path in self.image_paths:
            img_path = Path(img_path)
            if not img_path.is_file():
                self.get_logger().warn(f'Image not found: {img_path}')
                continue

            frame = cv2.imread(str(img_path))
            if frame is None:
                self.get_logger().warn(f'Failed to read image: {img_path}')
                continue

            img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = camera_name

            topic = f'{self._namespace}/{camera_name}/image'
            pub = self.create_publisher(Image, topic, _DEBUG_QOS)
            pub.publish(img_msg)
            self.get_logger().info(f'Published image on {topic}')

    # --- replay loop ------------------------------------------------------

    def replay(self) -> None:
        """Loop point cloud messages for the configured duration."""
        if not self._cloud_msgs:
            self.get_logger().warn('No point cloud messages to replay.')
            return

        period = 1.0 / self.rate_hz
        deadline = time.time() + self.duration_sec
        idx = 0
        total = len(self._cloud_msgs)

        self.get_logger().info(
            f'Replaying {total} clouds for {self.duration_sec}s on {self._cloud_topic}'
        )

        while rclpy.ok() and time.time() < deadline:
            msg = self._cloud_msgs[idx % total]
            # Update the timestamp so RViz doesn't discard as stale.
            msg.header.stamp = self.get_clock().now().to_msg()
            self._cloud_pub.publish(msg)
            idx += 1

            # Allow callbacks to process (e.g. for parameter updates).
            rclpy.spin_once(self, timeout_sec=period)

        self.get_logger().info('Replay complete.')


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def publish_dataset(
    dataset_name: str,
    bag_path: str | Path,
    image_paths: Optional[List[tuple[str, str | Path]]] = None,
    filter_bounds: Optional[Dict[str, float]] = None,
    duration_sec: float = 30.0,
    rate_hz: float = 10.0,
) -> bool:
    """Publish a collected dataset on debug topics for a given duration.

    Parameters
    ----------
    dataset_name:
        Name used in the topic namespace: /fast_calib/debug/<name>/pointcloud
    bag_path:
        Path to the rosbag2 bag directory.
    image_paths:
        Optional list of (camera_name, image_file_path) tuples.
    filter_bounds:
        Dict with x_min/x_max/y_min/y_max/z_min/z_max for the passthrough
        crop. If None, publishes the raw (unfiltered) cloud.
    duration_sec:
        How long to publish (loops the bag). Default 30 s.
    rate_hz:
        Point cloud publish rate. Default 10 Hz.

    Returns
    -------
    True if replay completed successfully, False on error.
    """
    own_init = not rclpy.ok()
    if own_init:
        rclpy.init()

    # Normalize image paths.
    normalized_images = [
        (name, Path(p)) for name, p in (image_paths or [])
    ]

    node = DatasetPublisher(
        dataset_name=dataset_name,
        bag_path=Path(bag_path),
        image_paths=normalized_images,
        filter_bounds=filter_bounds,
        duration_sec=duration_sec,
        rate_hz=rate_hz,
    )

    try:
        node.replay()
        return True
    except KeyboardInterrupt:
        node.get_logger().info('Replay interrupted.')
        return False
    finally:
        node.destroy_node()
        if own_init:
            rclpy.shutdown()
