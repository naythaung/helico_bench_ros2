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
    Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]),
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
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("meca500_scene_description"),
                    "rviz",
                    "scene.rviz",
                ]),
            ],
            additional_env={
                "QT_ENABLE_HIGHDPI_SCALING": "0",
            }
        ),
    ])