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
from launch.actions import (
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    gripper_urdf = PathJoinSubstitution(
        [FindPackageShare("prox2f_description"), "urdf", "gripper.urdf.xacro"]
    )
    gripper_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            gripper_urdf,
            " ",
            "name:=grp",
        ]
    )
    ghost_gripper_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            gripper_urdf,
            " ",
            "name:=grp_ghost",
            " ",
            "prefix:=grp_ghost_",
            " ",
            "ghost:=true",
        ]
    )
    gripper_description = {"robot_description": gripper_description_content}
    ghost_gripper_description = {"robot_description": ghost_gripper_description_content}

    gripper_launch = GroupAction(
        [
            PushRosNamespace("grp"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[gripper_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            # Grasp simulation
            PushRosNamespace("ghost"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[ghost_gripper_description],
            ),
            Node(
                package="prox2f_contact_analysis",
                executable="sim_state_publisher",
                remappings=[
                    ("input/points", "/proximity/concat/points"),
                    ("joint_states", "reference/joint_states"),
                ],
                parameters=[
                    {
                        "joint": "grp_ghost_left_finger1_joint",
                        "base_link": "tool0",
                    }
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"source_list": ["reference/joint_states"]}],
            ),
        ]
    )

    # Proximity sensors
    proximity_launch_file = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "launch", "proximity_sensors.launch.py"]
    )
    proximity_launch = GroupAction(
        [
            PushRosNamespace("proximity"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(proximity_launch_file),
                launch_arguments={
                    "left_sensor_namespace": "/vl53l5cx/x2a",
                    "right_sensor_namespace": "/vl53l5cx/x2b",
                    "concat_target_frame": "tool0",
                }.items(),
            ),
        ]
    )

    contact_analysis_nodes = []
    for finger in ("left", "right"):
        input_topic = f"/proximity/{finger}/points"
        namespace = f"contact_analysis/{finger}"
        surface_frame_id = f"{finger}_fingertip"

        contact_analysis_nodes.append(
            ComposableNode(
                package="prox2f_contact_analysis",
                plugin="prox::contact::ResampleCloud",
                namespace=namespace,
                remappings=[
                    ("input/points", input_topic),
                    ("points", "resampled/points"),
                ],
                parameters=[{"surface_frame_id": surface_frame_id}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

        contact_analysis_nodes.append(
            ComposableNode(
                package="prox2f_contact_analysis",
                plugin="prox::contact::ContactMapping",
                namespace=namespace,
                remappings=[
                    ("input/points", "resampled/points"),
                ],
                parameters=[{"penetration": 0.002}],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    contact_analysis_nodes.append(
        ComposableNode(
            package="prox2f_contact_analysis",
            plugin="prox::contact::VirtualWrench",
            namespace="contact_analysis",
            remappings=[
                ("input1/points", "left/points"),
                ("input2/points", "right/points"),
            ],
            parameters=[{"reference_frame_id": "tool0"}],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    contact_analysis_container = ComposableNodeContainer(
        name="contact_analysis_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=contact_analysis_nodes,
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "rviz", "view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        arguments=["-d", rviz_config_file],
        emulate_tty=True,
    )

    actions = [
        gripper_launch,
        contact_analysis_container,
        TimerAction(period=1.0, actions=[proximity_launch]),
        rviz_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[EmitEvent(event=Shutdown(reason="Window closed"))],
            )
        ),
    ]
    return LaunchDescription(actions)
