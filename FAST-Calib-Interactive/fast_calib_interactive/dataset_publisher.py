"""Replay a collected dataset on namespaced debug topics.

Reads a rosbag2 bag (mcap) and publishes its PointCloud2 messages on
``/fast_calib/debug/<dataset_name>/pointcloud``. If camera images exist for
the scene, they are published as well on
``/fast_calib/debug/<dataset_name>/<camera_name>/image``.

The replay loops continuously until the configured duration expires so that
downstream consumers (e.g. RViz) have time to visualize the data regardless of
how short the original recording was.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud2

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
        duration_sec: float = 30.0,
        rate_hz: float = 10.0,
    ):
        super().__init__('fast_calib_dataset_publisher')
        self.dataset_name = dataset_name
        self.bag_path = Path(bag_path)
        self.image_paths = image_paths or []
        self.duration_sec = duration_sec
        self.rate_hz = rate_hz

        self._namespace = f'{_TOPIC_PREFIX}/{dataset_name}'

        # Create the PointCloud2 publisher.
        self._cloud_topic = f'{self._namespace}/pointcloud'
        self._cloud_pub = self.create_publisher(
            PointCloud2, self._cloud_topic, _DEBUG_QOS
        )

        # Pre-load point cloud messages from the bag.
        self._cloud_msgs: List[PointCloud2] = []
        self._load_bag()

        # Publish images (latched via TRANSIENT_LOCAL).
        self._publish_images()

        self.get_logger().info(
            f'Dataset publisher ready:\n'
            f'  topic     : {self._cloud_topic}\n'
            f'  messages  : {len(self._cloud_msgs)} point clouds\n'
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
                self._cloud_msgs.append(msg)

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
