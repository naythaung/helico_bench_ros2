import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as file:
        return file.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)


def launch_setup(context):

    trajectory_name = LaunchConfiguration(
        "trajectory_name"
    ).perform(context)

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("meca500_scene_description"),
                "urdf",
                "scene.urdf.xacro",
            ]),
        ]),
        value_type=str,
    )

    robot_description = {
        "robot_description": robot_description_content
    }

    robot_description_semantic = {
        "robot_description_semantic":
            load_file(
                "meca500_moveit_config",
                "config/meca500.srdf",
            )
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
            load_yaml(
                "meca500_moveit_config",
                "config/kinematics.yaml",
            )
    }

    robot_description_planning = {
        "robot_description_planning":
            load_yaml(
                "meca500_moveit_config",
                "config/joint_limits.yaml",
            )
    }

    replay_node = Node(
        package="meca500_tasks",
        executable="replay_trajectory",
        output="screen",

        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,

            {
                "trajectory_name": trajectory_name,
            },
        ],
    )

    return [replay_node]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            "trajectory_name",
            default_value="test_demo",
            description="Name of saved trajectory to replay",
        ),

        OpaqueFunction(function=launch_setup),
    ])
