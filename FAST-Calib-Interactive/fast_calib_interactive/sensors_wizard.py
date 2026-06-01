"""Interactive wizard for creating a sensors.yaml from scratch.

Invoked automatically by the CLI when no sensors.yaml is found at the
specified path. Guides the user through defining cameras, LiDARs, and
calibration target geometry, then writes the file to disk.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import yaml


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

_TARGET_DEFAULTS: Dict[str, Any] = {
    'marker_size': 0.16,
    'delta_width_qr_center': 0.47,
    'delta_height_qr_center': 0.27,
    'delta_width_circles': 0.4,
    'delta_height_circles': 0.3,
    'circle_radius': 0.10,
    'min_detected_markers': 3,
}


# ---------------------------------------------------------------------------
# Prompt helpers
# ---------------------------------------------------------------------------

def _prompt(label: str, default: Any = None) -> str:
    suffix = f' [{default}]' if default is not None else ''
    raw = input(f'  {label}{suffix}: ').strip()
    return raw if raw else (str(default) if default is not None else '')


def _prompt_float(label: str, default: float) -> float:
    while True:
        raw = _prompt(label, default)
        try:
            return float(raw)
        except ValueError:
            print(f'  Please enter a number.')


def _prompt_int(label: str, default: int) -> int:
    while True:
        raw = _prompt(label, default)
        try:
            return int(raw)
        except ValueError:
            print(f'  Please enter an integer.')


def _prompt_nonempty(label: str, default: str | None = None) -> str:
    while True:
        val = _prompt(label, default)
        if val:
            return val
        print('  This field is required.')


# ---------------------------------------------------------------------------
# Section builders
# ---------------------------------------------------------------------------

def _collect_lidars() -> List[Dict[str, str]]:
    print('\n--- LiDARs ---')
    print('Define the LiDARs in your sensor suite.')
    lidars = []
    idx = 1
    while True:
        print(f'\n  LiDAR {idx}:')
        name = _prompt_nonempty('name', f'lidar_{idx}')
        topic = _prompt_nonempty('topic', '/livox/lidar')
        lidars.append({'name': name, 'topic': topic})
        idx += 1
        again = _prompt('Add another LiDAR? (y/N)', 'N').lower()
        if again != 'y':
            break
    return lidars


def _collect_cameras(lidar_names: List[str]) -> List[Dict[str, str]]:
    print('\n--- Cameras ---')
    print('Define the cameras to calibrate. Each camera must be paired with a LiDAR.')
    print(f'Available LiDARs: {", ".join(lidar_names)}')
    cameras = []
    idx = 1
    while True:
        print(f'\n  Camera {idx}:')
        name = _prompt_nonempty('name', f'cam_{idx}')
        image_topic = _prompt_nonempty('image_topic', f'/camera/{name}/image_raw')
        info_topic = _prompt_nonempty('info_topic', f'/camera/{name}/camera_info')

        # Validate lidar reference.
        while True:
            lidar = _prompt_nonempty('lidar (name from above)', lidar_names[0])
            if lidar in lidar_names:
                break
            print(f'  Unknown lidar "{lidar}". Choose from: {", ".join(lidar_names)}')

        cameras.append({
            'name': name,
            'image_topic': image_topic,
            'info_topic': info_topic,
            'lidar': lidar,
        })
        idx += 1
        again = _prompt('Add another camera? (y/N)', 'N').lower()
        if again != 'y':
            break
    return cameras


def _collect_target() -> Dict[str, Any]:
    print('\n--- Calibration Target ---')
    print('Enter target geometry. Press Enter to accept defaults.')
    return {
        'marker_size': _prompt_float(
            'marker_size (ArUco edge length, m)',
            _TARGET_DEFAULTS['marker_size'],
        ),
        'delta_width_qr_center': _prompt_float(
            'delta_width_qr_center (half horiz. marker spacing, m)',
            _TARGET_DEFAULTS['delta_width_qr_center'],
        ),
        'delta_height_qr_center': _prompt_float(
            'delta_height_qr_center (half vert. marker spacing, m)',
            _TARGET_DEFAULTS['delta_height_qr_center'],
        ),
        'delta_width_circles': _prompt_float(
            'delta_width_circles (horiz. circle spacing, m)',
            _TARGET_DEFAULTS['delta_width_circles'],
        ),
        'delta_height_circles': _prompt_float(
            'delta_height_circles (vert. circle spacing, m)',
            _TARGET_DEFAULTS['delta_height_circles'],
        ),
        'circle_radius': _prompt_float(
            'circle_radius (m)',
            _TARGET_DEFAULTS['circle_radius'],
        ),
        'min_detected_markers': _prompt_int(
            'min_detected_markers',
            _TARGET_DEFAULTS['min_detected_markers'],
        ),
    }


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------

def run_wizard(output_path: Path) -> Path:
    """Interactively build a sensors.yaml and write it to ``output_path``.

    Returns the path of the written file.
    """
    print('\n========================================')
    print('  sensors.yaml not found.')
    print('  Let\'s create one now.')
    print('========================================')

    lidars = _collect_lidars()
    lidar_names = [l['name'] for l in lidars]
    cameras = _collect_cameras(lidar_names)
    target = _collect_target()

    data = {
        'sensors': {
            'cameras': cameras,
            'lidars': lidars,
        },
        'target': target,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w') as fh:
        yaml.safe_dump(data, fh, default_flow_style=False, sort_keys=False)

    print(f'\n  sensors.yaml written to {output_path}')
    return output_path
