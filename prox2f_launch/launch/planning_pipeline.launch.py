#  Copyright 2024 Sakai Hibiki
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

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))
from prox2f_common import get_moveit_configs


def generate_launch_description():
    args = []
    args.append(DeclareLaunchArgument("robot_ip", default_value="192.168.11.180"))
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="false"))

    moveit_configs = get_moveit_configs()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_configs.to_dict()],
        arguments=["--ros-args", "--log-level", "warn"],
        output="screen",
        emulate_tty=True,
    )

    common_hybrid_planning_params = {
        "global_planning_action_name": "/hybrid_planning/global_planner",
        "local_planning_action_name": "/hybrid_planning/local_planner",
        "hybrid_planning_action_name": "/hybrid_planning/hybrid_planner",
    }

    global_planner_params = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "config", "global_planner.yaml"]
    )

    local_planner_params = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "config", "local_planner.yaml"]
    )

    hybrid_planning_manager_params = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "config", "hybrid_planning_manager.yaml"]
    )

    hybrid_planning_container = ComposableNodeContainer(
        name="hybrid_planning_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                parameters=[
                    common_hybrid_planning_params,
                    global_planner_params,
                    moveit_configs.robot_description,
                    moveit_configs.robot_description_semantic,
                    moveit_configs.robot_description_kinematics,
                    moveit_configs.joint_limits,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                parameters=[
                    common_hybrid_planning_params,
                    local_planner_params,
                    moveit_configs.robot_description,
                    moveit_configs.robot_description_semantic,
                    moveit_configs.robot_description_kinematics,
                    moveit_configs.joint_limits,
                ],
                remappings=[
                    ("~/pose", "grasp_pose_publisher/pose"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                parameters=[
                    common_hybrid_planning_params,
                    hybrid_planning_manager_params,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        emulate_tty=True,
        output="screen",
    )

    actions = [
        move_group_node,
        hybrid_planning_container,
    ]

    return LaunchDescription(args + actions)
