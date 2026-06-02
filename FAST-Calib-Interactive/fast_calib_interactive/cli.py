"""Interactive command-line front end for FAST-Calib.

This tool focuses on data collection and orchestration. All calibration math
lives in the ``fast_calib`` C++ package, which this CLI invokes unchanged.

Directory layout
----------------
$XDG_STATE_HOME/ros/fast_calib/
    scenes/<scene>/<camera>/lidar_bag/      recorded rosbag2 directories
    scenes/<scene>/<camera>/image.png       snapshot images
    configs/<scene>_<camera>_params.yaml    generated qr_params (state, not config)

--output <dir>  (default: ./fast_calib_output)
    <camera>/calib_result.txt
    <camera>/multi_calib_result.txt
    <camera>/colored_cloud.pcd
    <camera>/qr_detect.png
    inter_camera/T_<camB>_<camA>.txt        derived inter-camera transforms
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from .data_collector import CameraArtifacts, collect_lidar_group
from .sensor_config import SensorConfig
from .sensors_wizard import run_wizard
from .workflow_manager import PairScene, WorkflowManager

BANNER = r"""
==============================================
   FAST-Calib Interactive
   Collect scenes, then calibrate.
==============================================
"""


def _xdg_state_dir() -> Path:
    """Return $XDG_STATE_HOME/ros/fast_calib, creating it if needed."""
    xdg_state = Path(
        os.environ.get('XDG_STATE_HOME', Path.home() / '.local' / 'state')
    )
    state_dir = xdg_state / 'ros' / 'fast_calib'
    state_dir.mkdir(parents=True, exist_ok=True)
    return state_dir


def _xdg_config_sensors() -> Path:
    """Return $XDG_CONFIG_HOME/ros/fast_calib/sensors.yaml."""
    xdg_config = Path(
        os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')
    )
    return xdg_config / 'ros' / 'fast_calib' / 'sensors.yaml'


def _prompt(prompt: str, default: str | None = None) -> str:
    suffix = f' [{default}]' if default else ''
    raw = input(f'{prompt}{suffix}: ').strip()
    return raw or (default or '')


class InteractiveSession:
    """Drive the interactive menu loop."""

    def __init__(
        self,
        sensor_config: SensorConfig,
        output_path: Path,
        state_path: Path,
        log_level: str | None = None,
    ):
        self.sensor_config = sensor_config
        self.workflow = WorkflowManager(
            sensor_config=sensor_config,
            state_path=state_path,
            output_path=output_path,
            log_level=log_level,
        )

    # --- menu actions -----------------------------------------------------

    def collect(self) -> None:
        """Collect one scene for every camera-LiDAR pair in sensors.yaml.

        Cameras sharing the same LiDAR are recorded in a single pass so the
        LiDAR bag is only written once per unique LiDAR.
        """
        scene_name = _prompt(
            'Scene name',
            f'scene{len({s.scene_name for s in self.workflow.scenes}) + 1}',
        )
        num_msgs = int(_prompt('LiDAR frames to record per LiDAR', '10'))
        timeout = float(_prompt('Collection timeout per LiDAR group (sec)', '30'))

        # Group cameras by their associated LiDAR.
        lidar_groups: dict = {}
        for camera in self.sensor_config.cameras:
            lidar = self.sensor_config.lidar_for(camera)
            lidar_groups.setdefault(lidar, []).append(camera)

        scene_dir = self.workflow.state_path / 'scenes' / scene_name

        for lidar, cameras in lidar_groups.items():
            cam_names = ', '.join(c.name for c in cameras)
            print(f"\n  Recording LiDAR '{lidar.name}' ({lidar.topic})"
                  f" with cameras: {cam_names}")

            cam_tuples = [
                (c.name, c.image_topic, c.info_topic) for c in cameras
            ]
            artifacts = collect_lidar_group(
                scene_dir=scene_dir,
                lidar_name=lidar.name,
                lidar_topic=lidar.topic,
                cameras=cam_tuples,
                num_lidar_msgs=num_msgs,
                timeout_sec=timeout,
            )

            if artifacts is None:
                print(f"  WARNING: Collection timed out for LiDAR '{lidar.name}'. "
                      f"Skipping cameras: {cam_names}")
                continue

            for camera in cameras:
                if camera.name not in artifacts:
                    print(f"  WARNING: No artifacts for '{camera.name}', skipping.")
                    continue
                art = artifacts[camera.name]
                self.workflow.add_scene(PairScene(
                    scene_name=scene_name,
                    camera_name=camera.name,
                    bag=art.bag_path,
                    image=art.image_path,
                    intrinsics=art.intrinsics,
                ))
                print(f"  ✓ '{camera.name}' collected → {scene_dir / camera.name}")

    def list_scenes(self) -> None:
        if not self.workflow.scenes:
            print('No scenes collected yet.')
            return
        # Group by scene name for readability.
        by_scene: dict = {}
        for s in self.workflow.scenes:
            by_scene.setdefault(s.scene_name, []).append(s)
        for scene_name, pairs in by_scene.items():
            print(f'\n  {scene_name}:')
            for p in pairs:
                print(f'    [{p.camera_name}]  bag={p.bag}  image={p.image}')

    def calibrate_single(self) -> None:
        """Run single-scene calibration for the most recent scene, all cameras."""
        if not self.workflow.scenes:
            print('Collect at least one scene first.')
            return
        latest_scene = self.workflow.scenes[-1].scene_name
        pairs = [s for s in self.workflow.scenes if s.scene_name == latest_scene]
        for pair in pairs:
            print(f"\nCalibrating '{pair.camera_name}' (scene '{pair.scene_name}')...")
            if self.workflow.run_single_scene(pair):
                print(f"  ✓ Done.")
                result = self.workflow.latest_result(pair.camera_name)
                if result:
                    print('\n----- Result -----')
                    print(result)
                    print('------------------')
            else:
                print(f"  ✗ Failed. See output above.")

    def calibrate_multi(self) -> None:
        """Run multi-scene calibration for all cameras that have >= 3 scenes."""
        for camera in self.sensor_config.cameras:
            n = len(self.workflow.scenes_for(camera.name))
            if n < 3:
                print(
                    f"Skipping '{camera.name}': needs 3 scenes, have {n}."
                )
                continue
            print(f"\nRunning multi-scene calibration for '{camera.name}'...")
            if self.workflow.run_multi_scene(camera):
                print(f"  ✓ Done.")
                result = self.workflow.latest_result(camera.name)
                if result:
                    print('\n----- Result -----')
                    print(result)
                    print('------------------')
            else:
                print(f"  ✗ Failed. See output above.")

    def compute_inter_camera(self) -> None:
        """Derive inter-camera transforms from calibrated camera-LiDAR results."""
        print('\nComputing inter-camera transforms...')
        results = self.workflow.compute_inter_camera_transforms()
        if not results:
            print('No inter-camera transforms could be derived. '
                  'Ensure at least two cameras sharing a LiDAR have been calibrated.')
            return
        for key, T in results.items():
            print(f'\n  {key}:')
            print(T)

    # --- loop -------------------------------------------------------------

    def run(self) -> None:
        cameras = [c.name for c in self.sensor_config.cameras]
        lidars = [l.name for l in self.sensor_config.lidars]
        print(BANNER)
        print(f'  Cameras : {", ".join(cameras)}')
        print(f'  LiDARs  : {", ".join(lidars)}')
        print(f'  State   : {self.workflow.state_path}')
        print(f'  Output  : {self.workflow.output_path}')

        actions = {
            '1': ('Collect new scene (all cameras)', self.collect),
            '2': ('List collected scenes', self.list_scenes),
            '3': ('Run single-scene calibration (last scene)', self.calibrate_single),
            '4': ('Run multi-scene calibration', self.calibrate_multi),
            '5': ('Compute inter-camera transforms', self.compute_inter_camera),
            '6': ('Exit', None),
        }
        while True:
            print('\nMenu:')
            for key, (label, _) in actions.items():
                print(f'  {key}. {label}')
            choice = input('Select an option: ').strip()
            if choice == '6':
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
            except Exception as exc:  # noqa: BLE001
                print(f'Error: {exc}')


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Interactive CLI for FAST-Calib data collection and calibration.'
    )
    parser.add_argument(
        '--sensors',
        type=Path,
        default=_xdg_config_sensors(),
        help='Path to sensors.yaml '
             '(default: $XDG_CONFIG_HOME/ros/fast_calib/sensors.yaml). '
             'Created interactively if not found.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='Directory for final calibration results '
             '(default: $XDG_STATE_HOME/ros/fast_calib/result).',
    )
    parser.add_argument(
        '--state',
        type=Path,
        default=None,
        help=(
            'Directory for transient working data: bags, images, generated configs. '
            'Defaults to $XDG_STATE_HOME/ros/fast_calib.'
        ),
    )
    parser.add_argument(
        '--log-level',
        default=None,
        metavar='LEVEL',
        help=(
            'Log level passed to the fast_calib node '
            '(e.g. debug, info, warn). '
            'Use "debug" to see per-cluster rejection reasons. '
            'Default: ros default (info).'
        ),
    )
    args = parser.parse_args(argv)

    state_path = args.state if args.state is not None else _xdg_state_dir()
    output_path = args.output if args.output is not None else state_path / 'result'
    sensors_path = args.sensors

    if not sensors_path.is_file():
        try:
            sensors_path = run_wizard(sensors_path)
        except KeyboardInterrupt:
            print('\nAborted.', file=sys.stderr)
            return 1

    try:
        sensor_config = SensorConfig.from_file(sensors_path)
    except (FileNotFoundError, ValueError) as exc:
        print(f'Failed to load sensors.yaml: {exc}', file=sys.stderr)
        return 1

    session = InteractiveSession(sensor_config, output_path, state_path,
                                 log_level=args.log_level)
    session.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
