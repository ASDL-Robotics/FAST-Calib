"""Read and update the FAST-Calib ``qr_params.yaml`` configuration.

The interactive tool never reimplements calibration logic. It only edits the
parameter file that the ``fast_calib`` package already consumes, so the two
packages stay decoupled.
"""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict

import yaml

# Top-level keys expected in qr_params.yaml.
_ROOT_NODE = 'fast_calib'
_PARAMS_KEY = 'ros__parameters'


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
        if _ROOT_NODE not in self._data or _PARAMS_KEY not in self._data[_ROOT_NODE]:
            raise ValueError(
                f'Unexpected config structure in {self.config_path}; '
                f'expected top-level "{_ROOT_NODE}" -> "{_PARAMS_KEY}".'
            )
        return self._data

    @property
    def params(self) -> Dict[str, Any]:
        """Return the editable ``ros__parameters`` mapping."""
        return self._data[_ROOT_NODE][_PARAMS_KEY]

    def get(self, key: str, default: Any = None) -> Any:
        return self.params.get(key, default)

    def update(self, values: Dict[str, Any]) -> None:
        """Merge ``values`` into the parameter block."""
        self.params.update(values)

    def save(self, target_path: str | Path | None = None) -> Path:
        """Write the current config to disk.

        Returns the path that was written. ``target_path`` lets callers write a
        per-scene copy instead of mutating the shared template.
        """
        out_path = Path(target_path) if target_path is not None else self.config_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open('w') as handle:
            yaml.safe_dump(self._data, handle, default_flow_style=False, sort_keys=False)
        return out_path

    def clone(self) -> 'ConfigManager':
        """Return a deep-copied manager sharing no mutable state."""
        twin = ConfigManager.__new__(ConfigManager)
        twin.config_path = self.config_path
        twin._data = copy.deepcopy(self._data)
        return twin
