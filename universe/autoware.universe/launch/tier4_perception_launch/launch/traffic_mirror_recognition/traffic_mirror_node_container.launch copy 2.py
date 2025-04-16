# Copyright 2020 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    mirror_fine_detector_share_dir = get_package_share_directory("traffic_mirror_fine_detector")
    add_launch_arg("enable_image_decompressor", "True")
    add_launch_arg("enable_fine_detection", "True")
    add_launch_arg("input/image", "/sensing/camera/traffic_light/image_raw")
    add_launch_arg("output/rois", "/perception/traffic_light_recognition/rois")
    add_launch_arg(
        "output/traffic_signals", "/perception/traffic_light_recognition/traffic_signals"
    )

    # traffic_mirror_fine_detector
    add_launch_arg(
        "mirror_fine_detector_model_path",
        os.path.join(mirror_fine_detector_share_dir, "data", "tlr_yolox_s.onnx"),
    )
    add_launch_arg(
        "mirror_fine_detector_label_path",
        os.path.join(mirror_fine_detector_share_dir, "data", "tlr_labels.txt"),
    )
    add_launch_arg("mirror_fine_detector_precision", "fp16")
    add_launch_arg("mirror_fine_detector_score_thresh", "0.3")
    add_launch_arg("mirror_fine_detector_nms_thresh", "0.65")

    add_launch_arg("approximate_sync", "False")

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    container = ComposableNodeContainer(
        name="traffic_mirror_node_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable")
    )

    decompressor_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="image_transport_decompressor",
                plugin="image_preprocessor::ImageTransportDecompressor",
                name="traffic_light_image_decompressor",
                parameters=[{"encoding": "rgb8"}],
                remappings=[
                    (
                        "~/input/compressed_image",
                        [LaunchConfiguration("input/image"), "/compressed"],
                    ),
                    ("~/output/raw_image", LaunchConfiguration("input/image")),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_image_decompressor")),
    )

    mirror_fine_detector_param = create_parameter_dict(
        "mirror_fine_detector_model_path",
        "mirror_fine_detector_label_path",
        "mirror_fine_detector_precision",
        "mirror_fine_detector_score_thresh",
        "mirror_fine_detector_nms_thresh",
    )

    mirror_fine_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="traffic_mirror_fine_detector",
                plugin="traffic_mirror::TrafficMirrorFineDetectorNodelet",
                name="traffic_mirror_fine_detector",
                namespace="detection",
                parameters=[mirror_fine_detector_param],
                remappings=[
                    ("~/input/image", LaunchConfiguration("input/image")),
                    ("~/input/rois", "rough/rois"),
                    ("~/expect/rois", "expect/rois"),
                    ("~/output/rois", LaunchConfiguration("output/rois")),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_fine_detection")),
    )

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription(
        [
            *launch_arguments,
            set_container_executable,
            set_container_mt_executable,
            container,
            decompressor_loader,
            mirror_fine_detector_loader,
        ]
    )
