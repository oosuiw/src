import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory for adaptive_ndt_controller
    adaptive_ndt_controller_pkg_dir = get_package_share_directory('adaptive_ndt_controller')
    adaptive_ndt_controller_param_file = os.path.join(
        adaptive_ndt_controller_pkg_dir, 'config', 'adaptive_ndt_controller.param.yaml')

    # 1. pose_uncertainty_monitor node
    # This node calculates the uncertainty from the EKF output.
    pose_uncertainty_monitor_node = Node(
        package='pose_uncertainty_monitor',
        executable='pose_uncertainty_monitor_exe', # Make sure this executable name is correct
        name='pose_uncertainty_monitor',
        parameters=[{
            # Set the input topic to the correct, full topic name you found
            'input_pose_with_cov_topic': '/localization/pose_twist_fusion_filter/ekf_localizer/output/a_priori_pose_with_covariance',
            # Set the output topic that the controller will listen to
            'output_topic': '/adaptive_ndt/uncertainty_vector'
        }],
        output='screen'
    )

    # 2. adaptive_ndt_controller node
    # This node reads the uncertainty and controls the NDT parameters.
    adaptive_ndt_controller_node = Node(
        package='adaptive_ndt_controller',
        executable='adaptive_ndt_controller_exe',
        name='adaptive_ndt_controller',
        parameters=[adaptive_ndt_controller_param_file],
        output='screen'
    )

    return LaunchDescription([
        pose_uncertainty_monitor_node,
        adaptive_ndt_controller_node
    ])
