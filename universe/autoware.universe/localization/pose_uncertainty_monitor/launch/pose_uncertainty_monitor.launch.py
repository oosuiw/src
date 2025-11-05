from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pose_uncertainty_monitor')
    default_param_file = os.path.join(pkg_share, 'config/pose_uncertainty_monitor.param.yaml')

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to the parameter file for the pose uncertainty monitor.'
    )

    pose_uncertainty_monitor_node = Node(
        package='pose_uncertainty_monitor',
        executable='pose_uncertainty_monitor_exe',
        name='pose_uncertainty_monitor',
        parameters=[LaunchConfiguration('param_file')],
        output='screen',
    )

    return LaunchDescription([
        param_file_arg,
        pose_uncertainty_monitor_node
    ])
