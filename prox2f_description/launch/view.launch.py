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
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Descriptions
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("prox2f_description"), "urdf", "prox2f.urdf.xacro"]
            ),
            " ",
            "name:=prox2f",
        ]
    )
    ghost_gripper_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("prox2f_description"), "urdf", "gripper.urdf.xacro"]
            ),
            " ",
            "name:=grp_ghost",
            " ",
            "prefix:=grp_ghost_",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    ghost_gripper_description = {"robot_description": ghost_gripper_description_content}

    # robot_state_publisher nodes
    ur_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )
    gripper_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="ghost",
        parameters=[ghost_gripper_description],
    )

    # joint_state_publisher nodes
    ur_joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    gripper_joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace="ghost",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("prox2f_description"), "rviz", "view.rviz"]
            ),
        ],
        emulate_tty=True,
    )

    nodes = [
        ur_state_publisher_node,
        gripper_state_publisher_node,
        ur_joint_state_publisher_node,
        gripper_joint_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)
