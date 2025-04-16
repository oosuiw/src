import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    mirror_fine_detector_share_dir = get_package_share_directory("traffic_mirror_fine_detector")

    # === 원래 토픽 이름 유지 ===
    add_launch_arg("enable_image_decompressor", "True", "Enable image decompressor node")
    add_launch_arg("enable_fine_detection", "True", "Enable fine detection node")
    add_launch_arg("input/image", "/sensing/camera/traffic_light/image_raw", "Input image topic")
    add_launch_arg("output/rois", "/perception/traffic_mirror_recognition/rois", "Output ROIs topic")
    add_launch_arg("output/traffic_signals", "/perception/traffic_mirror_recognition/traffic_signals", "Output traffic signals topic")

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
    add_launch_arg("approximate_sync", "False", "Enable approximate time synchronization")
    add_launch_arg("use_intra_process", "False", "Use intra-process communication")
    add_launch_arg("use_multithread", "False", "Use multi-threading")

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

    # traffic_mirror_roi_visualizer 노드 (이미지 표시 및 ROI 시각화)
    roi_visualizer_node = ComposableNode(
        package="traffic_mirror_visualization",
        plugin="traffic_mirror::TrafficMirrorRoiVisualizerNodelet",
        name="traffic_mirror_roi_visualizer",
        parameters=[{"enable_fine_detection": LaunchConfiguration("enable_fine_detection")}],  # 수정
        remappings=[
            ("~/input/image", LaunchConfiguration("input/image")),
            ("~/input/rois", LaunchConfiguration("output/rois")),
            ("~/input/rough/rois", "~/input/rough/mirror_rois"),  # 수정
            ("~/input/traffic_signals", LaunchConfiguration("output/traffic_signals")),
            ("~/output/image", "~/debug/rois_mirror"),  # 수정
            ("~/output/image/compressed", "~/debug/rois/compressed_mirror"),  # 수정
            ("~/output/image/compressedDepth", "~/debug/rois/compressedDepth_mirror"),  # 수정
            ("~/output/image/theora", "~/debug/rois/theora_mirror"),  # 수정
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # 이미지 디컴프레서 노드
    decompressor_node = ComposableNode(
        package="image_transport_decompressor",
        plugin="image_preprocessor::ImageTransportDecompressor",
        name="traffic_mirror_image_decompressor",
        parameters=[{"encoding": "rgb8"}],
        remappings=[
            ("~/input/compressed_image", [LaunchConfiguration("input/image"), "/compressed"]),
            ("~/output/raw_image", LaunchConfiguration("input/image")),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # traffic_mirror_fine_detector 노드 (정밀 검출)
    fine_detector_node = ComposableNode(
        package="traffic_mirror_fine_detector",
        plugin="traffic_mirror::TrafficMirrorFineDetectorNodelet",
        name="traffic_mirror_fine_detector",
        namespace="detection",
        parameters=[mirror_fine_detector_param],
        remappings=[
            ("~/input/image", LaunchConfiguration("input/image")),  # 수정: 원본 이미지 사용
            ("~/input/rois", "~/input/rough/rois"),  # 수정: rough rois 입력
            ("~/output/rois", "/perception/traffic_mirror_recognition/rois"),   # 수정: fine rois 출력
            ("~/expect/rois", "~/expect/rois"), # 추가
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    # === 수정 2: ComposableNode 추가 방식 변경 ===
    # LoadComposableNodes를 사용하여 노드를 컨테이너에 추가
    container_loader = LoadComposableNodes(
        composable_node_descriptions=[
            roi_visualizer_node,
            decompressor_node,
            fine_detector_node,
        ],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("enable_fine_detection")), # enable_fine_detection 조건 추가
    )
    container_loader_no_fine_detection = LoadComposableNodes(
        composable_node_descriptions=[
            roi_visualizer_node,
            decompressor_node,
        ],
        target_container=container,
        condition=UnlessCondition(LaunchConfiguration("enable_fine_detection")), # enable_fine_detection 조건 추가
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
            container_loader,
            container_loader_no_fine_detection
        ]
    )