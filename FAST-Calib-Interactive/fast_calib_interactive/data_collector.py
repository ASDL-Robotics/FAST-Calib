"""Collect a calibration scene from live ROS 2 topics.

A scene is collected per LiDAR, not per camera. All cameras paired to the same
LiDAR are recorded simultaneously in a single node, sharing one bag. This avoids
recording redundant copies of the same LiDAR data when multiple cameras share a
LiDAR.

Artifacts written per scene
---------------------------
<scene_dir>/<lidar_name>/lidar_bag/     rosbag2 directory (one per LiDAR)
<scene_dir>/<camera_name>/image.png     one image per camera
<scene_dir>/<camera_name>/intrinsics    embedded in the returned dict
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

import rosbag2_py
from rclpy.serialization import serialize_message

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except ImportError:  # pragma: no cover - environment dependent
    _HAS_CV = False

_CAMERA_INFO_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ---------------------------------------------------------------------------
# Public result type
# ---------------------------------------------------------------------------

@dataclass
class CameraArtifacts:
    """Artifacts collected for one camera."""

    camera_name: str
    image_path: str
    bag_path: str           # shared with other cameras on the same LiDAR
    intrinsics: Dict[str, Any]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _extract_intrinsics(info: CameraInfo) -> Dict[str, Any]:
    """Extract fx/fy/cx/cy/k1/k2/p1/p2 from a CameraInfo message."""
    k = info.k
    d = list(info.d) + [0.0] * 4
    return {
        'fx': float(k[0]),
        'fy': float(k[4]),
        'cx': float(k[2]),
        'cy': float(k[5]),
        'k1': float(d[0]),
        'k2': float(d[1]),
        'p1': float(d[2]),
        'p2': float(d[3]),
    }


@dataclass
class _CameraState:
    """Per-camera mutable collection state."""

    name: str
    image_topic: str
    info_topic: str
    image_path: Path
    image_saved: bool = False
    intrinsics: Optional[Dict[str, Any]] = None

    @property
    def done(self) -> bool:
        return self.image_saved and self.intrinsics is not None


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MultiCameraSceneCollector(Node):
    """Record one LiDAR bag and images/intrinsics for all paired cameras.

    Parameters
    ----------
    scene_dir:
        Root directory for this scene's artifacts.
    lidar_name:
        Logical name of the LiDAR (used for the bag subdirectory).
    lidar_topic:
        ROS 2 topic for the LiDAR PointCloud2 stream.
    cameras:
        List of (camera_name, image_topic, info_topic) tuples.
    num_lidar_msgs:
        Number of LiDAR frames to record before stopping.
    """

    def __init__(
        self,
        scene_dir: Path,
        lidar_name: str,
        lidar_topic: str,
        cameras: List[tuple[str, str, str]],
        num_lidar_msgs: int = 10,
    ):
        super().__init__('fast_calib_scene_collector')
        self.scene_dir = Path(scene_dir)
        self.lidar_topic = lidar_topic
        self.num_lidar_msgs = num_lidar_msgs

        self.bag_path = self.scene_dir / lidar_name / 'lidar_bag'
        self.bag_path.parent.mkdir(parents=True, exist_ok=True)

        self._lidar_count = 0
        self._bridge = CvBridge() if _HAS_CV else None
        self._writer = self._open_bag_writer()

        # Build per-camera state objects.
        self._cameras: Dict[str, _CameraState] = {}
        for cam_name, image_topic, info_topic in cameras:
            image_path = self.scene_dir / cam_name / 'image.png'
            image_path.parent.mkdir(parents=True, exist_ok=True)
            self._cameras[cam_name] = _CameraState(
                name=cam_name,
                image_topic=image_topic,
                info_topic=info_topic,
                image_path=image_path,
            )

        # Subscribe to LiDAR.
        self._lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self._on_lidar, _SENSOR_QOS
        )

        # Subscribe to each camera's image and info topics.
        for state in self._cameras.values():
            self.create_subscription(
                Image, state.image_topic,
                lambda msg, s=state: self._on_image(msg, s),
                _SENSOR_QOS,
            )
            self.create_subscription(
                CameraInfo, state.info_topic,
                lambda msg, s=state: self._on_camera_info(msg, s),
                _CAMERA_INFO_QOS,
            )

        cam_list = ', '.join(self._cameras)
        self.get_logger().info(
            f'Collecting scene into {self.scene_dir}\n'
            f'  lidar   : {self.lidar_topic} ({num_lidar_msgs} frames)\n'
            f'  cameras : {cam_list}'
        )

    # --- bag writer -------------------------------------------------------

    def _open_bag_writer(self) -> rosbag2_py.SequentialWriter:
        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            ),
        )
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=self.lidar_topic,
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr',
        ))
        return writer

    # --- callbacks --------------------------------------------------------

    def _on_lidar(self, msg: PointCloud2) -> None:
        if self._lidar_count >= self.num_lidar_msgs:
            return
        stamp = self.get_clock().now().nanoseconds
        self._writer.write(self.lidar_topic, serialize_message(msg), stamp)
        self._lidar_count += 1
        self.get_logger().info(
            f'LiDAR frame {self._lidar_count}/{self.num_lidar_msgs} recorded.'
        )

    def _on_image(self, msg: Image, state: _CameraState) -> None:
        if state.image_saved:
            return
        if not _HAS_CV:
            self.get_logger().error('cv_bridge/opencv not available.')
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(str(state.image_path), frame)
            state.image_saved = True
            self.get_logger().info(
                f'[{state.name}] Image saved to {state.image_path}.'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[{state.name}] Failed to save image: {exc}')

    def _on_camera_info(self, msg: CameraInfo, state: _CameraState) -> None:
        if state.intrinsics is not None:
            return
        state.intrinsics = _extract_intrinsics(msg)
        intr = state.intrinsics
        self.get_logger().info(
            f'[{state.name}] Intrinsics: '
            f'fx={intr["fx"]:.2f} fy={intr["fy"]:.2f} '
            f'cx={intr["cx"]:.2f} cy={intr["cy"]:.2f}'
        )

    # --- completion -------------------------------------------------------

    @property
    def done(self) -> bool:
        lidar_done = self._lidar_count >= self.num_lidar_msgs
        cameras_done = all(s.done for s in self._cameras.values())
        return lidar_done and cameras_done

    def close(self) -> None:
        del self._writer

    def results(self) -> Dict[str, CameraArtifacts]:
        """Return per-camera artifacts. Call only after ``done`` is True."""
        return {
            name: CameraArtifacts(
                camera_name=name,
                image_path=str(state.image_path),
                bag_path=str(self.bag_path),
                intrinsics=state.intrinsics,
            )
            for name, state in self._cameras.items()
            if state.done
        }


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def collect_lidar_group(
    scene_dir: str | Path,
    lidar_name: str,
    lidar_topic: str,
    cameras: List[tuple[str, str, str]],
    num_lidar_msgs: int = 10,
    timeout_sec: float = 30.0,
) -> Optional[Dict[str, CameraArtifacts]]:
    """Record one LiDAR bag and all paired camera images simultaneously.

    Parameters
    ----------
    scene_dir:
        Root directory for this scene.
    lidar_name:
        Logical LiDAR name (used for the bag subdirectory).
    lidar_topic:
        LiDAR PointCloud2 topic.
    cameras:
        List of ``(camera_name, image_topic, info_topic)`` tuples.
    num_lidar_msgs:
        LiDAR frames to record.
    timeout_sec:
        Abort if not complete within this many seconds.

    Returns
    -------
    Dict mapping camera_name → CameraArtifacts, or None on timeout.
    """
    own_init = not rclpy.ok()
    if own_init:
        rclpy.init()

    node = MultiCameraSceneCollector(
        scene_dir=Path(scene_dir),
        lidar_name=lidar_name,
        lidar_topic=lidar_topic,
        cameras=cameras,
        num_lidar_msgs=num_lidar_msgs,
    )
    deadline = time.time() + timeout_sec
    try:
        while rclpy.ok() and not node.done and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        success = node.done
        artifacts = node.results()
        node.close()
    finally:
        node.destroy_node()
        if own_init:
            rclpy.shutdown()

    return artifacts if success else None
