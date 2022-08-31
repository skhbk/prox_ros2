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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Gripper
    gripper_launch_file = PathJoinSubstitution([
        FindPackageShare('robotiq_description'),
        'launch',
        'bringup.launch.py'
    ])
    gripper_launch = GroupAction([
        PushRosNamespace('robotiq'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gripper_launch_file)
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        # Grasp simulation
        PushRosNamespace('sim'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gripper_launch_file),
            launch_arguments={
                'prefix': 'sim_',
            }.items(),
        ),
        Node(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
    ])

    # Proximity sensors
    proximity_launch_file = PathJoinSubstitution([
        FindPackageShare('prox2f_launch'),
        'launch',
        'proximity_sensors.launch.py'
    ])
    proximity_launch = GroupAction([
        PushRosNamespace('proximity'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(proximity_launch_file),
            launch_arguments={
                'left_sensor_namespace': '/vl53l5cx/x2a',
                'right_sensor_namespace': '/vl53l5cx/x2b',
            }.items()
        ),
    ])

    # Transforms for proximity sensors
    static_transform_publisher_nodes = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.015', '0', '0.045', '-1.57079', '-1.57079', '1.57079',
                       'robotiq_85_left_finger_tip_link', 'proximity/left']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.015', '0', '0.045', '1.57079', '-1.57079', '1.57079',
                       'robotiq_85_right_finger_tip_link', 'proximity/right']
        ),
    ]

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('prox2f_launch'), 'rviz', 'view_robot.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace='',
        arguments=['-d', rviz_config_file],
        # remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        emulate_tty=True,
    )

    actions = []
    actions.extend(static_transform_publisher_nodes)
    actions.append(gripper_launch)
    # Delay starting the nodes to wait for transforms
    actions.append(TimerAction(period=1., actions=[proximity_launch]))
    actions.append(rviz_node)

    return LaunchDescription(actions)
