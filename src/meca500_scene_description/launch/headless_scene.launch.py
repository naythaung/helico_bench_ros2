from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("meca500_scene_description"), "urdf", "scene.urdf.xacro"]
    )
    robot_description_content = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
    ])