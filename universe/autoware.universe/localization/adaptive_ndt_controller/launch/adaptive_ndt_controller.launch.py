import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('adaptive_ndt_controller')
    param_file = os.path.join(pkg_dir, 'config', 'adaptive_ndt_controller.param.yaml')

    adaptive_ndt_controller_node = Node(
        package='adaptive_ndt_controller',
        executable='adaptive_ndt_controller_node_exe',
        name='adaptive_ndt_controller',
        parameters=[param_file],
        output='screen',
        remappings=[
            # Remap the uncertainty topic if needed
            # ("input/pose_uncertainty", "/your/custom/topic"),
        ]
    )

    return LaunchDescription([
        adaptive_ndt_controller_node
    ])
