from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("meca500_description"), "urdf", "meca500.urdf.xacro"]
    )
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file]
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
            package="rviz2",
            executable="rviz2",
            output="log",
        ),
        Node(
            package="meca500_driver",
            executable="joint_state_bridge",
            output="screen",
        ),
        Node(
            package="camera_bridge",
            executable="stream_subscriber",
            output="screen",
        ),
    ])