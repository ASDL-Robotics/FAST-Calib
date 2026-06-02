"""Orchestrate FAST-Calib runs without touching its source.

The manager keeps a list of collected scenes, writes the appropriate paths into
a per-scene copy of ``qr_params.yaml``, and shells out to the existing
``fast_calib`` / ``multi_fast_calib`` executables via ``ros2``.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

from .config_manager import ConfigManager


@dataclass
class Scene:
    """A single collected calibration scene."""

    name: str
    bag: str
    image: str


@dataclass
class WorkflowManager:
    """Track scenes and dispatch calibration runs."""

    config_path: Path
    output_path: Path
    scenes: List[Scene] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.config_path = Path(self.config_path)
        self.output_path = Path(self.output_path)
        self.output_path.mkdir(parents=True, exist_ok=True)

    def add_scene(self, scene: Scene) -> None:
        self.scenes.append(scene)

    def _write_scene_config(self, scene: Scene) -> Path:
        """Create a scene-specific config derived from the template."""
        cfg = ConfigManager(self.config_path)
        cfg.update(
            {
                'bag_path': scene.bag,
                'image_path': scene.image,
                'output_path': str(self.output_path),
            }
        )
        scene_cfg_dir = self.output_path / 'configs'
        scene_cfg_dir.mkdir(parents=True, exist_ok=True)
        scene_cfg_path = scene_cfg_dir / f'{scene.name}_params.yaml'
        return cfg.save(scene_cfg_path)

    @staticmethod
    def _run(cmd: List[str]) -> int:
        """Run a subprocess, streaming output, returning its exit code."""
        print(f'\n$ {" ".join(cmd)}\n')
        completed = subprocess.run(cmd, check=False)
        return completed.returncode

    def run_single_scene(self, scene: Scene) -> bool:
        """Run single-scene calibration for one collected scene."""
        cfg_path = self._write_scene_config(scene)
        rc = self._run(
            [
                'ros2', 'run', 'fast_calib', 'fast_calib',
                '--ros-args', '--params-file', str(cfg_path),
            ]
        )
        return rc == 0

    def run_multi_scene(self) -> bool:
        """Run multi-scene calibration over accumulated records.

        ``multi_fast_calib`` reads ``circle_center_record.txt`` from the output
        directory, which is populated by prior single-scene runs, so we only need
        to point it at the same output path.
        """
        if len(self.scenes) < 3:
            print(f'Need at least 3 scenes for multi-scene (have {len(self.scenes)}).')
            return False

        cfg = ConfigManager(self.config_path)
        cfg.update({'output_path': str(self.output_path)})
        cfg_path = cfg.save(self.output_path / 'configs' / 'multi_params.yaml')

        rc = self._run(
            [
                'ros2', 'run', 'fast_calib', 'multi_fast_calib',
                '--ros-args', '--params-file', str(cfg_path),
            ]
        )
        return rc == 0

    def latest_result(self) -> Optional[str]:
        """Return the contents of the most relevant result file, if present."""
        multi = self.output_path / 'multi_calib_result.txt'
        single = self.output_path / 'calib_result.txt'
        for candidate in (multi, single):
            if candidate.is_file():
                return candidate.read_text()
        return None
