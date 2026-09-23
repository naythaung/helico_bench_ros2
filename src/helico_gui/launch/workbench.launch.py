from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ---------------------------------------------------------
    # Planning action server
    # ---------------------------------------------------------

    planning_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("meca500_tasks"),
                    "launch",
                    "plan_trajectory_server.launch.py",
                ]
            )
        )
    )

    # ---------------------------------------------------------
    # Execution action server
    # ---------------------------------------------------------

    execution_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("meca500_tasks"),
                    "launch",
                    "trajectory_execution_server.launch.py",
                ]
            )
        )
    )

    # ---------------------------------------------------------
    # Pose / trajectory library server
    # ---------------------------------------------------------

    workbench_server = Node(
        package="meca500_tasks",
        executable="workbench_server",
        name="meca500_workbench_server",
        output="screen",
    )

    # ---------------------------------------------------------
    # Bench sensor controller
    # ---------------------------------------------------------

    sensor_bridge = Node(
        package="helico_sensors",
        executable="serial_bridge",
        name="helico_sensor_bridge",
        output="screen",
        parameters=[
            {
                "port": "auto",
            }
        ],
    )

    # ---------------------------------------------------------
    # Helico controller
    # ---------------------------------------------------------

    actuator_bridge = Node(
        package="helico_sensors",
        executable="actuator_bridge",
        name="helico_actuator_bridge",
        output="screen",
        parameters=[
            {
                "port": "auto",
            }
        ],
    )

    # ---------------------------------------------------------
    # GUI
    # ---------------------------------------------------------

    gui = Node(
        package="helico_gui",
        executable="helico_qt",
        name="helico_qt",
        output="screen",
    )

    return LaunchDescription(
        [
            planning_server,
            execution_server,
            workbench_server,
            sensor_bridge,
            actuator_bridge,
            gui,
        ]
    )