"""Parse and validate the sensors.yaml configuration file.

sensors.yaml defines which cameras and LiDARs to calibrate and how they are
paired. Intrinsics are intentionally absent — they are fetched at runtime from
each camera's CameraInfo topic.

Example sensors.yaml
--------------------
sensors:
  cameras:
    - name: cam_front
      image_topic: /camera/front/image_raw
      info_topic:  /camera/front/camera_info
      lidar: lidar_main

    - name: cam_left
      image_topic: /camera/left/image_raw
      info_topic:  /camera/left/camera_info
      lidar: lidar_main

  lidars:
    - name: lidar_main
      topic: /livox/lidar

target:
  marker_size: 0.16
  delta_width_qr_center: 0.47
  delta_height_qr_center: 0.27
  delta_width_circles: 0.4
  delta_height_circles: 0.3
  circle_radius: 0.10
  min_detected_markers: 3
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml

# Required keys for each section.
_CAMERA_REQUIRED = {'name', 'image_topic', 'info_topic', 'lidar'}
_LIDAR_REQUIRED = {'name', 'topic'}
_TARGET_REQUIRED = {
    'marker_size',
    'delta_width_qr_center',
    'delta_height_qr_center',
    'delta_width_circles',
    'delta_height_circles',
    'circle_radius',
    'min_detected_markers',
}


@dataclass(frozen=True)
class CameraConfig:
    """Configuration for a single camera."""

    name: str
    image_topic: str
    info_topic: str
    lidar: str          # name of the associated LiDAR entry


@dataclass(frozen=True)
class LidarConfig:
    """Configuration for a single LiDAR."""

    name: str
    topic: str


@dataclass(frozen=True)
class TargetConfig:
    """Calibration target geometry, shared across all camera-LiDAR pairs."""

    marker_size: float
    delta_width_qr_center: float
    delta_height_qr_center: float
    delta_width_circles: float
    delta_height_circles: float
    circle_radius: float
    min_detected_markers: int

    def as_dict(self) -> Dict[str, Any]:
        return {
            'marker_size': self.marker_size,
            'delta_width_qr_center': self.delta_width_qr_center,
            'delta_height_qr_center': self.delta_height_qr_center,
            'delta_width_circles': self.delta_width_circles,
            'delta_height_circles': self.delta_height_circles,
            'circle_radius': self.circle_radius,
            'min_detected_markers': self.min_detected_markers,
        }


@dataclass
class SensorConfig:
    """Parsed and validated contents of sensors.yaml."""

    cameras: List[CameraConfig]
    lidars: List[LidarConfig]
    target: TargetConfig

    # Convenience lookup maps built on construction.
    _lidar_map: Dict[str, LidarConfig] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        self._lidar_map = {l.name: l for l in self.lidars}

    def lidar_for(self, camera: CameraConfig) -> LidarConfig:
        """Return the LidarConfig associated with a camera."""
        if camera.lidar not in self._lidar_map:
            raise ValueError(
                f"Camera '{camera.name}' references unknown lidar '{camera.lidar}'. "
                f"Known lidars: {list(self._lidar_map)}"
            )
        return self._lidar_map[camera.lidar]

    @classmethod
    def from_file(cls, path: str | Path) -> 'SensorConfig':
        """Load and validate a sensors.yaml file."""
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(f'Sensor config not found: {path}')

        with path.open('r') as fh:
            raw = yaml.safe_load(fh) or {}

        # --- cameras ---
        raw_cameras = raw.get('sensors', {}).get('cameras', [])
        if not raw_cameras:
            raise ValueError('sensors.yaml must define at least one camera.')
        cameras = []
        for i, cam in enumerate(raw_cameras):
            missing = _CAMERA_REQUIRED - cam.keys()
            if missing:
                raise ValueError(
                    f'Camera entry {i} is missing required keys: {missing}'
                )
            cameras.append(CameraConfig(**{k: cam[k] for k in _CAMERA_REQUIRED}))

        # --- lidars ---
        raw_lidars = raw.get('sensors', {}).get('lidars', [])
        if not raw_lidars:
            raise ValueError('sensors.yaml must define at least one lidar.')
        lidars = []
        for i, lid in enumerate(raw_lidars):
            missing = _LIDAR_REQUIRED - lid.keys()
            if missing:
                raise ValueError(
                    f'Lidar entry {i} is missing required keys: {missing}'
                )
            lidars.append(LidarConfig(**{k: lid[k] for k in _LIDAR_REQUIRED}))

        # --- target ---
        raw_target = raw.get('target', {})
        missing = _TARGET_REQUIRED - raw_target.keys()
        if missing:
            raise ValueError(f'target section is missing required keys: {missing}')
        target = TargetConfig(**{k: raw_target[k] for k in _TARGET_REQUIRED})

        return cls(cameras=cameras, lidars=lidars, target=target)
