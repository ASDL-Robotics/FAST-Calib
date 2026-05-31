"""Launch the FAST-Calib interactive CLI.

Note: this is an interactive terminal program. Launching it via ``ros2 launch``
works, but running the console script directly is usually more convenient:

    ros2 run fast_calib_interactive interactive_calib
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output_arg = DeclareLaunchArgument(
        'output',
        default_value='fast_calib_interactive_output',
        description='Directory for collected scenes and calibration outputs.',
    )

    interactive_proc = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'fast_calib_interactive', 'interactive_calib',
            '--output', LaunchConfiguration('output'),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([output_arg, interactive_proc])
