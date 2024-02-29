from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def get_moveit_configs():
    robot_description_mappings = {
        "robot_ip": LaunchConfiguration("robot_ip"),
        "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
        "kinematics_params": PathJoinSubstitution(
            [
                FindPackageShare("prox2f_description"),
                "config",
                "ur5e_kinematics.yaml",
            ]
        ),
        "initial_positions_file": PathJoinSubstitution(
            [
                FindPackageShare("prox2f_description"),
                "config",
                "ur_initial_positions.yaml",
            ]
        ),
    }

    moveit_configs = (
        MoveItConfigsBuilder("prox2f")
        .robot_description(mappings=robot_description_mappings)
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )

    return moveit_configs
