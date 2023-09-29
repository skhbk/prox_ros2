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
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    args = []
    args.append(DeclareLaunchArgument("robot_ip", default_value="192.168.11.180"))
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="false"))
    args.append(
        DeclareLaunchArgument("kinematics_file", default_value="ur5e_kinematics.yaml")
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("prox2f_description"), "urdf", "prox2f.urdf.xacro"]
            ),
            " ",
            "robot_ip:=",
            LaunchConfiguration("robot_ip"),
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " ",
            "kinematics_params:=",
            PathJoinSubstitution(
                [
                    FindPackageShare("prox2f_description"),
                    "config",
                    LaunchConfiguration("kinematics_file"),
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_controllers = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "config", "prox2f_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_controllers],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("use_fake_hardware")),
    )

    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[robot_description, initial_controllers],
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": LaunchConfiguration("robot_ip")}],
    )

    tool_communication_node = Node(
        package="ur_robot_driver",
        executable="tool_communication.py",
        name="ur_tool_comm",
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
    )

    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
        parameters=[
            {"headless_mode": False},
            {"joint_controller_active": False},
            {"consistent_controllers": ["joint_state_broadcaster"]},
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    twist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "-c", "/controller_manager", "--inactive"],
    )

    gripper_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_command_controller", "-c", "/controller_manager"],
        condition=UnlessCondition(LaunchConfiguration("use_fake_hardware")),
    )

    # Rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("prox2f_launch"), "rviz", "view.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "warn"],
        emulate_tty=True,
    )

    # VL53L5CX
    activate_proximity = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"), "lifecycle set /vl53l5cx activate"],
        shell=True,
    )
    deactivate_proximity = ExecuteProcess(
        cmd=[FindExecutable(name="ros2"), "lifecycle set /vl53l5cx deactivate"],
        shell=True,
    )

    actions = [
        tool_communication_node,
        dashboard_client_node,
        # Wait for tool communication to be established
        TimerAction(
            period=1.0,
            actions=[
                control_node,
                ur_control_node,
                controller_stopper_node,
                robot_state_publisher_node,
                joint_state_broadcaster_spawner,
                twist_controller_spawner,
                gripper_command_controller_spawner,
            ],
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node, activate_proximity],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[EmitEvent(event=Shutdown(reason="Window closed"))],
            )
        ),
        RegisterEventHandler(
            event_handler=OnShutdown(on_shutdown=[deactivate_proximity])
        ),
    ]

    return LaunchDescription(args + actions)
