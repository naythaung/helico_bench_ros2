import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return file.read()

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    
    # 1. Load the URDF from your description package
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare("meca500_scene_description"), "urdf", "scene.urdf.xacro"])
        ]),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Load the SRDF
    robot_description_semantic = {
        "robot_description_semantic": load_file("meca500_moveit_config", "config/meca500.srdf")
    }

    # 3. Load Kinematics
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml("meca500_moveit_config", "config/kinematics.yaml")
    }
    
    # 3.5 Load Joint Limits (Crucial for time parameterization!)
    robot_description_planning = {
        "robot_description_planning": load_yaml("meca500_moveit_config", "config/joint_limits.yaml")
    }
    
    # 4. Load OMPL Planning Pipeline (This calculates the timestamps!)
    ompl_yaml = load_yaml("meca500_moveit_config", "config/ompl_planning.yaml")
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",

        "ompl.planning_plugins": ompl_yaml["planning_plugins"],
        "ompl.request_adapters": ompl_yaml["request_adapters"],
        "ompl.response_adapters": ompl_yaml["response_adapters"],
        "ompl.start_state_max_bounds_error": ompl_yaml["start_state_max_bounds_error"],
    }

    planning_pipeline.update({
        "ompl.meca_arm.default_planner_config":
            ompl_yaml["meca_arm"]["default_planner_config"],

        "ompl.meca_arm.planner_configs":
            list(ompl_yaml["meca_arm"]["planner_configs"].keys()),
    })

    for planner_name, planner_config in ompl_yaml["meca_arm"]["planner_configs"].items():
        for key, value in planner_config.items():
            planning_pipeline[f"ompl.planner_configs.{planner_name}.{key}"] = value

    # 4. Load Controllers configuration
    controllers_yaml = load_yaml("meca500_moveit_config", "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": controllers_yaml["moveit_simple_controller_manager"],
    }

    # 5. Trajectory execution settings — prevent premature timeout on real hardware
    trajectory_execution = {
        "trajectory_execution": {
            "allowed_execution_duration_scaling": 1.5,
            "allowed_goal_duration_margin": 1.0,
            "execution_duration_monitoring": True,
        }
    }

    # 6. Define the Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            moveit_controllers,
            trajectory_execution,
            {"use_sim_time": False},
        ],
    )

    # 7. Start RViz2 with MoveIt parameters
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
         "-d",
         os.path.join(
             get_package_share_directory("meca500_moveit_config"),
             "config",
             "scene.rviz",
         ),
     ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
        additional_env={
            "QT_ENABLE_HIGHDPI_SCALING": "0",
        },
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    mock_controllers_path = os.path.join(
        get_package_share_directory("meca500_moveit_config"),
        "config",
        "mock_controllers.yaml",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[mock_controllers_path],
        remappings=[("robot_description", "/robot_description")],
        output="screen",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "meca_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", mock_controllers_path,
        ],
        output="screen",
    )

    return LaunchDescription([
        run_move_group_node,
        robot_state_publisher_node,
        control_node,
        controller_spawner,
        TimerAction(
            period=8.0,
            actions=[rviz_node],
        ),
    ])