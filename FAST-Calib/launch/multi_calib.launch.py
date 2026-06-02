from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz'
    )

    pkg_share = get_package_share_directory('fast_calib')
    params_file = os.path.join(pkg_share, 'config', 'qr_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz_cfg', 'fast_livo2.rviz')

    multi_calib_node = Node(
        package='fast_calib',
        executable='multi_fast_calib',
        name='multi_fast_calib',
        parameters=[params_file],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        multi_calib_node,
        rviz_node
    ])
