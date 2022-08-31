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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.extend([
        DeclareLaunchArgument('left_sensor_namespace', default_value='left'),
        DeclareLaunchArgument('right_sensor_namespace', default_value='right'),
        DeclareLaunchArgument('concat_target_frame', default_value='world'),
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

    composable_nodes = []
    for input_namespace, output_namespace in zip(input_namespaces, output_namespaces):
        # EMA filter
        composable_nodes.append(ComposableNode(
            package='prox_preprocess',
            plugin='prox::EMA',
            namespace=output_namespace,
            remappings=[
                    ('input/image', [input_namespace, '/image']),
                    ('image', 'ema/image'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))
        # Convert to point cloud
        composable_nodes.append(ComposableNode(
            package='depth_image_proc',
            plugin='depth_image_proc::PointCloudXyzNode',
            namespace=output_namespace,
            remappings=[
                    ('image_rect', 'ema/image'),
                    ('camera_info', [input_namespace, '/camera_info']),
                    ('points', 'raw/points'),
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))
        # Preprocess point cloud
        composable_nodes.append(ComposableNode(
            package='prox_preprocess',
            plugin='prox::CloudProcessor',
            namespace=output_namespace,
            remappings=[
                ('input/points', 'raw/points'),
            ],
            parameters=[{
                'pass_through/field_name': 'z',
                'pass_through/limit_min': .01,
                'pass_through/limit_max': .08,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        ))

    # Concatenate point clouds
    composable_nodes.append(ComposableNode(
        package='prox_preprocess',
        plugin='prox::ConcatenateClouds',
        remappings=[
            ('input1/points', output_namespaces[0] + '/points'),
            ('input2/points', output_namespaces[1] + '/points'),
            ('points', 'concat/points'),
        ],
        parameters=[{
            'target_frame_id': LaunchConfiguration('concat_target_frame'),
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    ))

    component_container = ComposableNodeContainer(
        name='proximity_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'warn'],
    )

    # Automatically start/stop ranging
    vl53l5cx_client_node = Node(
        package='vl53l5cx_client',
        executable='auto_control',
    )

    actions = []
    actions.extend(declared_arguments)
    actions.append(component_container)
    actions.append(vl53l5cx_client_node)

    return LaunchDescription(actions)
