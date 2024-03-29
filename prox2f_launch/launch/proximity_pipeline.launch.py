#  Copyright 2022 Sakai Hibiki
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    args = []
    args.append(
        DeclareLaunchArgument(
            "left_sensor_namespace", default_value="/vl53l5cx_left/x29"
        )
    )
    args.append(
        DeclareLaunchArgument(
            "right_sensor_namespace", default_value="/vl53l5cx_right/x29"
        )
    )

    input_namespaces = [
        LaunchConfiguration("left_sensor_namespace"),
        LaunchConfiguration("right_sensor_namespace"),
    ]
    output_namespaces = (
        "left",
        "right",
    )

    containers = []

    for input_namespace, output_namespace in zip(input_namespaces, output_namespaces):
        composable_nodes = []
        # Image smoothing
        composable_nodes.append(
            ComposableNode(
                package="prox_preprocess",
                plugin="prox::preprocess::ImageSmoothing",
                namespace=output_namespace,
                remappings=[("input/image", [input_namespace, "/image"])],
                parameters=[{"filter_coefficient": 0.3, "lower_clip": 0.01}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Convert to point cloud
        composable_nodes.append(
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzNode",
                namespace=output_namespace,
                remappings=[
                    ("image_rect", "image_smoothing/image"),
                    ("camera_info", [input_namespace, "/camera_info"]),
                    ("points", "raw_points"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Preprocess point cloud
        composable_nodes.append(
            ComposableNode(
                package="prox_preprocess",
                plugin="prox::preprocess::CloudProcessor",
                namespace=output_namespace,
                remappings=[
                    ("input/points", "raw_points"),
                ],
                parameters=[
                    {"pass_through.field_name": "z", "pass_through.bounds": [0.0, 0.3]}
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Triangulation
        composable_nodes.append(
            ComposableNode(
                package="prox_mesh",
                plugin="prox::mesh::Triangulation",
                namespace=output_namespace,
                remappings=[("input/points", "cloud_processor/points")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Mesh visualization
        composable_nodes.append(
            ComposableNode(
                package="prox_mesh",
                plugin="prox::mesh::MeshToMarker",
                namespace=output_namespace,
                remappings=[("input/mesh_stamped", "triangulation/mesh_stamped")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Normals visualization
        composable_nodes.append(
            ComposableNode(
                package="prox_mesh",
                plugin="prox::mesh::NormalsToMarker",
                namespace=output_namespace,
                remappings=[("input/normals", "triangulation/normals")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Container
        containers.append(
            ComposableNodeContainer(
                name="proximity_container",
                namespace=output_namespace,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=composable_nodes,
                emulate_tty=True,
                arguments=["--ros-args", "--log-level", "warn"],
            )
        )

    grasp_pose_publisher_node = ComposableNode(
        package="prox2f_pregrasp",
        plugin="prox::pregrasp::GraspPosePublisher",
        remappings=[
            ("input1/normals", [output_namespaces[0], "/triangulation/normals"]),
            ("input2/normals", [output_namespaces[1], "/triangulation/normals"]),
        ],
        parameters=[{"output_frame_id": "tcp", "tcp_frame_id": "tcp"}],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    containers.append(
        ComposableNodeContainer(
            name="pregrasp_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                grasp_pose_publisher_node,
            ],
            emulate_tty=True,
            arguments=["--ros-args", "--log-level", "warn"],
        )
    )

    actions = [*containers]

    return LaunchDescription(args + actions)
