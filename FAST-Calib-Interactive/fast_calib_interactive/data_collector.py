"""Collect a single calibration scene from live ROS 2 topics.

A "scene" is the pair of inputs FAST-Calib expects:
  * a rosbag2 directory containing LiDAR ``PointCloud2`` messages, and
  * a single camera image saved to disk.

The collector subscribes to the configured topics, waits until it has captured
the requested number of LiDAR messages, grabs one synchronized image, and writes
both artifacts into a per-scene directory.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2

import rosbag2_py
from rclpy.serialization import serialize_message

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except ImportError:  # pragma: no cover - environment dependent
    _HAS_CV = False


class SceneCollector(Node):
    """ROS 2 node that records one calibration scene to disk."""

    def __init__(
        self,
        scene_dir: Path,
        lidar_topic: str,
        image_topic: str,
        num_lidar_msgs: int = 10,
    ):
        super().__init__('fast_calib_scene_collector')
        self.scene_dir = Path(scene_dir)
        self.scene_dir.mkdir(parents=True, exist_ok=True)

        self.lidar_topic = lidar_topic
        self.image_topic = image_topic
        self.num_lidar_msgs = num_lidar_msgs

        self.bag_path = self.scene_dir / 'lidar_bag'
        self.image_path = self.scene_dir / 'image.png'

        self._lidar_count = 0
        self._image_saved = False
        self._bridge = CvBridge() if _HAS_CV else None

        # Sensor data is best-effort / high-throughput.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._writer = self._open_bag_writer()

        self._lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self._on_lidar, sensor_qos
        )
        self._image_sub = self.create_subscription(
            Image, self.image_topic, self._on_image, sensor_qos
        )

        self.get_logger().info(
            f'Collecting scene into {self.scene_dir} '
            f'(lidar="{self.lidar_topic}", image="{self.image_topic}").'
        )

    def _open_bag_writer(self) -> rosbag2_py.SequentialWriter:
        writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(self.bag_path), storage_id='sqlite3'
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        writer.open(storage_options, converter_options)
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=self.lidar_topic,
                type='sensor_msgs/msg/PointCloud2',
                serialization_format='cdr',
            )
        )
        return writer

    def _on_lidar(self, msg: PointCloud2) -> None:
        if self._lidar_count >= self.num_lidar_msgs:
            return
        stamp = self.get_clock().now().nanoseconds
        self._writer.write(self.lidar_topic, serialize_message(msg), stamp)
        self._lidar_count += 1
        self.get_logger().info(
            f'LiDAR frame {self._lidar_count}/{self.num_lidar_msgs} recorded.'
        )

    def _on_image(self, msg: Image) -> None:
        if self._image_saved:
            return
        if not _HAS_CV:
            self.get_logger().error(
                'cv_bridge/opencv not available; cannot save image.'
            )
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(str(self.image_path), frame)
            self._image_saved = True
            self.get_logger().info(f'Image saved to {self.image_path}.')
        except Exception as exc:  # noqa: BLE001 - report and keep node alive
            self.get_logger().error(f'Failed to save image: {exc}')

    @property
    def done(self) -> bool:
        return self._lidar_count >= self.num_lidar_msgs and self._image_saved

    def close(self) -> None:
        """Finalize the bag writer."""
        # rosbag2_py closes on destruction; explicit del flushes sqlite.
        del self._writer


def collect_scene(
    scene_dir: str | Path,
    lidar_topic: str,
    image_topic: str,
    num_lidar_msgs: int = 10,
    timeout_sec: float = 30.0,
) -> Optional[dict]:
    """Record one scene and return its artifact paths.

    Returns a dict with ``bag`` and ``image`` keys on success, or ``None`` if the
    collection timed out before capturing everything.
    """
    own_init = not rclpy.ok()
    if own_init:
        rclpy.init()

    node = SceneCollector(scene_dir, lidar_topic, image_topic, num_lidar_msgs)
    deadline = time.time() + timeout_sec
    try:
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        success = node.done
        node.close()
    finally:
        node.destroy_node()
        if own_init:
            rclpy.shutdown()

    if not success:
        return None
    return {'bag': str(node.bag_path), 'image': str(node.image_path)}
