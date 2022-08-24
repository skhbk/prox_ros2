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
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    gripper_launch = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/robotiq_2f_85.launch.py']
            )
        )
    ])

    proximity_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/proximity_sensors.launch.py']
        ),
        launch_arguments={
            'left_sensor_namespace': '/vl53l5cx/x2a',
            'right_sensor_namespace': '/vl53l5cx/x2b'
        }.items()
    )

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

    actions = [
        gripper_launch,
        proximity_launch,
        rviz_node,
    ]

    return LaunchDescription(actions)
