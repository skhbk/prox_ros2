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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.event_handlers import OnShutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    args = []
    args.extend(
        [
            DeclareLaunchArgument("left_sensor_namespace", default_value="left"),
            DeclareLaunchArgument("right_sensor_namespace", default_value="right"),
            DeclareLaunchArgument("concat_target_frame", default_value="world"),
        ]
    )

    # Initialize Arguments
    input_namespaces = (
        LaunchConfiguration("left_sensor_namespace"),
        LaunchConfiguration("right_sensor_namespace"),
    )
    output_namespaces = (
        "left",
        "right",
    )

    composable_nodes = []
    for input_namespace, output_namespace in zip(input_namespaces, output_namespaces):
        # Image smoothing
        composable_nodes.append(
            ComposableNode(
                package="prox_preprocess",
                plugin="prox::ImageSmoothing",
                namespace=output_namespace,
                remappings=[
                    ("input/image", [input_namespace, "/image"]),
                    ("image", "smooth/image"),
                ],
                parameters=[
                    {
                        "weight": 0.2,
                    }
                ],
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
                    ("image_rect", "smooth/image"),
                    ("camera_info", [input_namespace, "/camera_info"]),
                    ("points", "raw/points"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
        # Preprocess point cloud
        composable_nodes.append(
            ComposableNode(
                package="prox_preprocess",
                plugin="prox::CloudProcessor",
                namespace=output_namespace,
                remappings=[
                    ("input/points", "raw/points"),
                ],
                parameters=[
                    {
                        "pass_through/field_name": "z",
                        "pass_through/limit_min": 0.01,
                        "pass_through/limit_max": 0.08,
                        "outlier/radius": 0.007,
                        "outlier/min_neighbors": 3,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    # Concatenate point clouds
    composable_nodes.append(
        ComposableNode(
            package="prox_preprocess",
            plugin="prox::ConcatenateClouds",
            remappings=[
                ("input1/points", output_namespaces[0] + "/points"),
                ("input2/points", output_namespaces[1] + "/points"),
                ("points", "concat/points"),
            ],
            parameters=[
                {
                    "target_frame_id": LaunchConfiguration("concat_target_frame"),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    component_container = ComposableNodeContainer(
        name="proximity_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "warn"],
    )

    start_ranging = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service call",
            "/vl53l5cx/start_ranging",
            "std_srvs/srv/Empty",
        ],
        shell=True,
    )
    stop_ranging = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service call",
            "/vl53l5cx/stop_ranging",
            "std_srvs/srv/Empty",
        ],
        shell=True,
    )

    actions = [
        component_container,
        start_ranging,
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[stop_ranging],
            )
        ),
    ]

    return LaunchDescription(args + actions)
