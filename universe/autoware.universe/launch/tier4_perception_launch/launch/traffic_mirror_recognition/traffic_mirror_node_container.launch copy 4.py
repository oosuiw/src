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
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    mirror_fine_detector_share_dir = get_package_share_directory("traffic_mirror_fine_detector")
    add_launch_arg("enable_image_decompressor", "True")
    add_launch_arg("enable_fine_detection", "True")
    add_launch_arg("input/image", "/sensing/camera/traffic_light/image_raw")
    add_launch_arg("output/rois", "/perception/traffic_mirror_recognition/rois")
    add_launch_arg("output/traffic_signals", "/perception/traffic_mirror_recognition/traffic_signals")

    add_launch_arg(
        "fine_detector_model_path",
        os.path.join(mirror_fine_detector_share_dir, "data", "yolox.onnx"),
        "Path to the fine detector ONNX model"
    )
    add_launch_arg(
        "fine_detector_label_path",
        os.path.join(mirror_fine_detector_share_dir, "data", "tmr_labels.txt"),
        "Path to the fine detector label file"
    )
    add_launch_arg("mirror_fine_detector_precision", "fp16")
    add_launch_arg("mirror_fine_detector_score_thresh", "0.3")
    add_launch_arg("mirror_fine_detector_nms_thresh", "0.65")
    add_launch_arg("approximate_sync", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_multithread", "False")

    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    # fine_detector 파라미터 정의
    mirror_fine_detector_param = create_parameter_dict(
        "fine_detector_model_path",
        "fine_detector_label_path",
        "mirror_fine_detector_precision",
        "mirror_fine_detector_score_thresh",
        "mirror_fine_detector_nms_thresh",
    )

    # 컨테이너 정의 (기본 노드 없이 빈 컨테이너로 시작)
    container = ComposableNodeContainer(
        name="traffic_mirror_node_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        output="both",  # 로그 출력을 활성화하여 디버깅 용이
    )

    # 이미지 디컴프레서 노드
    decompressor_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="image_transport_decompressor",
                plugin="image_preprocessor::ImageTransportDecompressor",
                name="traffic_mirror_image_decompressor",
                parameters=[{"encoding": "rgb8"}],
                remappings=[
                    ("~/input/compressed_image", [LaunchConfiguration("input/image"), "/compressed"]),
                    ("~/output/raw_image", LaunchConfiguration("input/image")),
                ],
                extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
            ),
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_image_decompressor")),
    )

    # traffic_mirror_fine_detector 노드 추가
    fine_detector_loader = LoadComposableNodes(
        composable_node_descriptions=[
            ComposableNode(
                package="traffic_mirror_fine_detector",
                plugin="traffic_mirror::TrafficMirrorFineDetectorNodelet",
                name="traffic_mirror_fine_detector",
                namespace="detection",
                parameters=[mirror_fine_detector_param],
                remappings=[
                    ("~/input/image", "/sensing/camera/traffic_light/image_raw"),
                    ("~/input/rois", "/perception/traffic_mirror_recognition/traffic_mirror/detection/rough/rois"),
                    ("~/expect/rois", "/perception/traffic_mirror_recognition/traffic_mirror/detection/expect/rois"),
                    ("~/output/rois", "/perception/traffic_mirror_recognition/traffic_mirror/detection/rois"),
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
            fine_detector_loader,  # traffic_mirror_fine_detector 추가
        ]
    )