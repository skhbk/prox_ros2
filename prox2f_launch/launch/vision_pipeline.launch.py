#  Copyright 2023 Sakai Hibiki
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = []

    camera_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name="camera",
        parameters=[
            {
                "pointcloud.enable": True,
                "pointcloud.stream_filter": 2,
                "colorizer.enable": True,
                "enable_sync": True,
                "align_depth.enable": True,
                "enable_infra1": False,
                "enable_infra2": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera_transform_publisher_node = ComposableNode(
        package="prox2f_camera",
        plugin="prox::camera::CameraTransformPublisher",
        remappings=[
            ("~/image_raw", "/color/image_raw"),
            ("~/camera_info", "/color/camera_info"),
        ],
        parameters=[{"marker_frame_id": "aruco0"}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    camera_container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            camera_node,
            camera_transform_publisher_node,
        ],
        emulate_tty=True,
    )

    object_detection_node = Node(
        package="prox2f_detection",
        executable="object_detection_node",
        remappings=[
            ("/depth/image_raw", "/aligned_depth_to_color/image_raw"),
            ("/depth/camera_info", "/aligned_depth_to_color/camera_info"),
        ],
        parameters=[
            {
                "model_path": PathJoinSubstitution(
                    [FindPackageShare("prox2f_detection"), "models", "ycb4.pt"]
                ),
                "confidence_threshold": 0.4,
                "iou_threshold": 0.6,
            }
        ],
        emulate_tty=True,
    )

    actions = [camera_container, object_detection_node]

    return LaunchDescription(args + actions)
