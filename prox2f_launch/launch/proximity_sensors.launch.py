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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # This namespace is pushed later
    namespace = 'proximity'

    # Declare arguments
    declared_arguments = []
    declared_arguments.extend([
        DeclareLaunchArgument('left_sensor_namespace', default_value='left'),
        DeclareLaunchArgument('right_sensor_namespace', default_value='right'),
    ])

    # Initialize Arguments
    input_namespaces = (
        LaunchConfiguration('left_sensor_namespace'),
        LaunchConfiguration('right_sensor_namespace'),
    )
    output_namespaces = (
        'left',
        'right',
    )

    # e.g. proximity/left
    frame_ids = [namespace + '/' + x for x in output_namespaces]

    static_transform_publisher_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.01', '0.003', '0', '-1.57079', '1.57079',
                       'left_inner_finger_pad', frame_ids[0]],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.01', '0.003', '0', '-1.57079', '1.57079',
                       'right_inner_finger_pad', frame_ids[1]],
        ),
    ]

    composable_nodes = []
    # Convert to point cloud
    for input_namespace, output_namespace in zip(input_namespaces, output_namespaces):
        composable_nodes.append(ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            namespace=output_namespace,
            remappings=[
                    ('image_rect', [input_namespace, '/image']),
                    ('camera_info', [input_namespace, '/camera_info']),
            ],
        ))
    # Concatenate point clouds
    composable_nodes.append(ComposableNode(
        package='prox_preprocess',
        plugin='prox::ConcatenateClouds',
        remappings=[
            ('input1/points', output_namespaces[0] + '/points'),
            ('input2/points', output_namespaces[1] + '/points'),
        ],
    ))

    component_container = ComposableNodeContainer(
        name='proximity_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
    )

    actions = []
    actions.append(PushRosNamespace(namespace))  # Push namsepace
    actions.extend(declared_arguments)
    actions.extend(static_transform_publisher_nodes)
    actions.append(component_container)

    return LaunchDescription([GroupAction(actions)])
