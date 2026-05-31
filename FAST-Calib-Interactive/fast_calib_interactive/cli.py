"""Interactive command-line front end for FAST-Calib.

This tool focuses on data collection and orchestration. All calibration math
lives in the ``fast_calib`` C++ package, which this CLI invokes unchanged.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from .config_manager import ConfigManager
from .data_collector import collect_scene
from .workflow_manager import Scene, WorkflowManager

BANNER = r"""
==============================================
   FAST-Calib Interactive
   Collect scenes, then calibrate.
==============================================
"""


def _default_config_path() -> Path:
    """Locate the installed fast_calib qr_params.yaml, if available."""
    try:
        share = get_package_share_directory('fast_calib')
        return Path(share) / 'config' / 'qr_params.yaml'
    except PackageNotFoundError:
        return Path('qr_params.yaml')


def _prompt(prompt: str, default: str | None = None) -> str:
    suffix = f' [{default}]' if default else ''
    raw = input(f'{prompt}{suffix}: ').strip()
    return raw or (default or '')


class InteractiveSession:
    """Drive the interactive menu loop."""

    def __init__(self, config_path: Path, output_path: Path):
        self.config = ConfigManager(config_path)
        self.workflow = WorkflowManager(
            config_path=config_path, output_path=output_path
        )

    # --- menu actions -----------------------------------------------------

    def collect(self) -> None:
        name = _prompt('Scene name', f'scene{len(self.workflow.scenes) + 1}')
        lidar_topic = _prompt(
            'LiDAR topic', self.config.get('lidar_topic', '/livox/lidar')
        )
        image_topic = _prompt('Image topic', '/camera/image_raw')
        num_msgs = int(_prompt('Number of LiDAR frames to record', '10'))

        scene_dir = self.workflow.output_path / 'scenes' / name
        print(f'\nGet the calibration target in view, then collection starts...')
        artifacts = collect_scene(
            scene_dir=scene_dir,
            lidar_topic=lidar_topic,
            image_topic=image_topic,
            num_lidar_msgs=num_msgs,
        )
        if artifacts is None:
            print('Collection timed out. No scene was added.')
            return

        self.workflow.add_scene(
            Scene(name=name, bag=artifacts['bag'], image=artifacts['image'])
        )
        print(f"Scene '{name}' collected.")

    def list_scenes(self) -> None:
        if not self.workflow.scenes:
            print('No scenes collected yet.')
            return
        print('\nCollected scenes:')
        for idx, scene in enumerate(self.workflow.scenes, start=1):
            print(f'  {idx}. {scene.name}')
            print(f'       bag:   {scene.bag}')
            print(f'       image: {scene.image}')

    def calibrate_single(self) -> None:
        if not self.workflow.scenes:
            print('Collect at least one scene first.')
            return
        scene = self.workflow.scenes[-1]
        print(f"Running single-scene calibration for '{scene.name}'...")
        if self.workflow.run_single_scene(scene):
            print('Single-scene calibration finished.')
            self._show_result()
        else:
            print('Single-scene calibration failed. See output above.')

    def calibrate_multi(self) -> None:
        print('Running multi-scene calibration...')
        if self.workflow.run_multi_scene():
            print('Multi-scene calibration finished.')
            self._show_result()
        else:
            print('Multi-scene calibration did not complete.')

    def _show_result(self) -> None:
        result = self.workflow.latest_result()
        if result:
            print('\n----- Calibration result -----')
            print(result)
            print('-------------------------------')

    # --- loop -------------------------------------------------------------

    def run(self) -> None:
        print(BANNER)
        actions = {
            '1': ('Collect new scene', self.collect),
            '2': ('List collected scenes', self.list_scenes),
            '3': ('Run single-scene calibration (last scene)', self.calibrate_single),
            '4': ('Run multi-scene calibration', self.calibrate_multi),
            '5': ('Exit', None),
        }
        while True:
            print('\nMenu:')
            for key, (label, _) in actions.items():
                print(f'  {key}. {label}')
            choice = input('Select an option: ').strip()
            if choice == '5':
                print('Goodbye.')
                return
            entry = actions.get(choice)
            if entry is None:
                print('Invalid option.')
                continue
            try:
                entry[1]()
            except KeyboardInterrupt:
                print('\nInterrupted. Returning to menu.')
            except Exception as exc:  # noqa: BLE001 - keep session alive
                print(f'Error: {exc}')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Interactive CLI for FAST-Calib data collection and calibration.'
    )
    parser.add_argument(
        '--config',
        type=Path,
        default=_default_config_path(),
        help='Path to FAST-Calib qr_params.yaml (defaults to installed fast_calib config).',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path.cwd() / 'fast_calib_interactive_output',
        help='Directory for collected scenes and calibration outputs.',
    )
    args = parser.parse_args(argv)

    try:
        session = InteractiveSession(args.config, args.output)
    except (FileNotFoundError, ValueError) as exc:
        print(f'Failed to start: {exc}', file=sys.stderr)
        return 1

    session.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
