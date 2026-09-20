import os

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import (
    get_package_share_directory,
)

import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(
        package_name
    )

    absolute_path = os.path.join(
        package_path,
        file_path,
    )

    with open(absolute_path, "r") as file:
        return file.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(
        package_name
    )

    absolute_path = os.path.join(
        package_path,
        file_path,
    )

    with open(absolute_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():

    # ---------------------------------------------------------
    # Robot URDF
    # ---------------------------------------------------------

    robot_description_content = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(
                            "meca500_scene_description"
                        ),
                        "urdf",
                        "scene.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    robot_description = {
        "robot_description":
            robot_description_content
    }

    # ---------------------------------------------------------
    # MoveIt SRDF
    # ---------------------------------------------------------

    robot_description_semantic = {
        "robot_description_semantic":
            load_file(
                "meca500_moveit_config",
                "config/meca500.srdf",
            )
    }

    # ---------------------------------------------------------
    # Kinematics
    # ---------------------------------------------------------

    robot_description_kinematics = {
        "robot_description_kinematics":
            load_yaml(
                "meca500_moveit_config",
                "config/kinematics.yaml",
            )
    }

    # ---------------------------------------------------------
    # Joint limits
    # ---------------------------------------------------------

    robot_description_planning = {
        "robot_description_planning":
            load_yaml(
                "meca500_moveit_config",
                "config/joint_limits.yaml",
            )
    }

    # ---------------------------------------------------------
    # Action server
    # ---------------------------------------------------------

    plan_trajectory_server = Node(
        package="meca500_tasks",
        executable="plan_trajectory_server",
        name="meca500_plan_trajectory_server",
        output="screen",

        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    return LaunchDescription(
        [
            plan_trajectory_server,
        ]
    )
