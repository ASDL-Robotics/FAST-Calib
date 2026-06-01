"""Read, build, and persist FAST-Calib ``qr_params.yaml`` files.

Two usage modes:

1. Load an existing template (e.g. the installed fast_calib default) and patch
   specific keys before saving a per-scene copy.

2. Build a complete config from scratch using live intrinsics from CameraInfo
   and target geometry from sensors.yaml — no template required.

All generated configs are written into the state directory
($XDG_STATE_HOME/ros/fast_calib) so they never pollute the source tree.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

_SINGLE_NODE = 'mono_qr_pattern'
_MULTI_NODE = 'multi_fast_calib'
_PARAMS_KEY = 'ros__parameters'


def _wrap(params: Dict[str, Any], node_name: str = _SINGLE_NODE) -> Dict[str, Any]:
    """Wrap a flat params dict in the ROS 2 YAML node structure."""
    return {node_name: {_PARAMS_KEY: params}}


class ConfigManager:
    """Load, inspect, and persist a FAST-Calib parameter file."""

    def __init__(self, config_path: str | Path):
        self.config_path = Path(config_path)
        self._data: Dict[str, Any] = {}
        self.load()

    def load(self) -> Dict[str, Any]:
        """Read the YAML file from disk into memory."""
        if not self.config_path.is_file():
            raise FileNotFoundError(f'Config file not found: {self.config_path}')
        with self.config_path.open('r') as handle:
            self._data = yaml.safe_load(handle) or {}
        # Accept any single top-level node name (mono_qr_pattern, multi_fast_calib, etc.)
        node_keys = [k for k in self._data if _PARAMS_KEY in (self._data[k] or {})]
        if not node_keys:
            raise ValueError(
                f'Unexpected config structure in {self.config_path}; '
                f'expected a top-level node name with a "{_PARAMS_KEY}" block.'
            )
        self._node_name = node_keys[0]
        return self._data

    @property
    def params(self) -> Dict[str, Any]:
        """Return the editable ``ros__parameters`` mapping."""
        return self._data[self._node_name][_PARAMS_KEY]

    def get(self, key: str, default: Any = None) -> Any:
        return self.params.get(key, default)

    def update(self, values: Dict[str, Any]) -> None:
        """Merge ``values`` into the parameter block."""
        self.params.update(values)

    def save(self, target_path: Optional[str | Path] = None) -> Path:
        """Write the current config to disk and return the path written."""
        out_path = Path(target_path) if target_path is not None else self.config_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open('w') as handle:
            yaml.safe_dump(
                self._data, handle, default_flow_style=False, sort_keys=False
            )
        return out_path

    def clone(self) -> 'ConfigManager':
        """Return a deep-copied manager sharing no mutable state."""
        twin = ConfigManager.__new__(ConfigManager)
        twin.config_path = self.config_path
        twin._data = copy.deepcopy(self._data)
        twin._node_name = self._node_name
        return twin

    # ------------------------------------------------------------------
    # Factory: build a complete config from live data (no template needed)
    # ------------------------------------------------------------------

    @classmethod
    def build(
        cls,
        *,
        intrinsics: Dict[str, Any],
        target: Dict[str, Any],
        bag_path: str,
        image_path: str,
        output_path: str,
        lidar_topic: str,
        save_path: str | Path,
        filter_bounds: Optional[Dict[str, float]] = None,
        node_name: str = _SINGLE_NODE,
    ) -> 'ConfigManager':
        """Construct a ConfigManager from live intrinsics and target geometry.

        Parameters
        ----------
        intrinsics:
            Dict with keys fx, fy, cx, cy, k1, k2, p1, p2 — typically from
            a CameraInfo message via data_collector.collect_scene().
        target:
            Dict with calibration target geometry keys from sensors.yaml.
        bag_path:
            Path to the recorded rosbag2 directory.
        image_path:
            Path to the saved camera image.
        output_path:
            Directory where fast_calib should write its results.
        lidar_topic:
            LiDAR topic name as recorded in the bag.
        save_path:
            Where to write the generated qr_params.yaml (should be under
            $XDG_STATE_HOME/ros/fast_calib/configs/).
        filter_bounds:
            Optional dict with x_min/x_max/y_min/y_max/z_min/z_max.
            Defaults to a wide passthrough if omitted.
        """
        defaults: Dict[str, float] = {
            'x_min': -10.0, 'x_max': 10.0,
            'y_min': -10.0, 'y_max': 10.0,
            'z_min': -10.0, 'z_max': 10.0,
        }
        bounds = {**defaults, **(filter_bounds or {})}

        params: Dict[str, Any] = {
            # Camera intrinsics (from CameraInfo)
            'fx': intrinsics['fx'],
            'fy': intrinsics['fy'],
            'cx': intrinsics['cx'],
            'cy': intrinsics['cy'],
            'k1': intrinsics['k1'],
            'k2': intrinsics['k2'],
            'p1': intrinsics['p1'],
            'p2': intrinsics['p2'],
            # Target geometry (from sensors.yaml)
            **target,
            # I/O paths
            'bag_path': bag_path,
            'image_path': image_path,
            'output_path': output_path,
            'lidar_topic': lidar_topic,
            # Distance filter
            **bounds,
        }

        instance = cls.__new__(cls)
        instance.config_path = Path(save_path)
        instance._node_name = node_name
        instance._data = _wrap(params, node_name)
        instance.save(save_path)
        return instance
