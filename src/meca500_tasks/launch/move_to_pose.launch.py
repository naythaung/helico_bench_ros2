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

    # ---------------------------------------------------------
    # 1. Launch arguments
    # ---------------------------------------------------------

    pose_name = LaunchConfiguration(
        "pose"
    ).perform(context)

    trajectory_name = LaunchConfiguration(
        "trajectory_name"
    ).perform(context)

    num_candidates = int(
        LaunchConfiguration(
            "num_candidates"
        ).perform(context)
    )

    minimum_required_clearance = float(
        LaunchConfiguration(
            "minimum_required_clearance"
        ).perform(context)
    )

    weight_clearance = float(
        LaunchConfiguration(
            "weight_clearance"
        ).perform(context)
    )

    weight_smoothness = float(
        LaunchConfiguration(
            "weight_smoothness"
        ).perform(context)
    )

    weight_path_length = float(
        LaunchConfiguration(
            "weight_path_length"
        ).perform(context)
    )

    weight_duration = float(
        LaunchConfiguration(
            "weight_duration"
        ).perform(context)
    )

    # ---------------------------------------------------------
    # 2. Load saved target pose
    # ---------------------------------------------------------

    poses_path = os.path.join(
        get_package_share_directory("meca500_tasks"),
        "config",
        "poses.yaml",
    )

    with open(poses_path, "r") as file:
        poses = yaml.safe_load(file)

    if pose_name not in poses:
        raise RuntimeError(
            f"Unknown pose '{pose_name}'. "
            f"Available poses: {list(poses.keys())}"
        )

    pose = poses[pose_name]

    if "type" not in pose:
        raise RuntimeError(
            f"Pose '{pose_name}' has no 'type'. "
            f"Expected 'cartesian' or 'joint'."
        )

    pose_type = pose["type"]

    if pose_type not in ["cartesian", "joint"]:
        raise RuntimeError(
            f"Pose '{pose_name}' has invalid type "
            f"'{pose_type}'."
        )

    print(f"Selected saved pose: {pose_name}")
    print(f"Pose type: {pose_type}")
    print(pose)

    print(
        "Trajectory planning settings:"
    )

    print(
        f"  Candidates        : {num_candidates}"
    )

    print(
        "  Minimum clearance : "
        f"{minimum_required_clearance * 1000.0:.1f} mm"
    )

    print(
        "  Weights           : "
        f"C={weight_clearance:.2f} "
        f"S={weight_smoothness:.2f} "
        f"L={weight_path_length:.2f} "
        f"T={weight_duration:.2f}"
    )

    # ---------------------------------------------------------
    # 3. Load same robot model as main MoveIt launch
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

    # ---------------------------------------------------------
    # 4. Start move_to_pose node
    # ---------------------------------------------------------

    move_node = Node(
        package="meca500_tasks",
        executable="move_to_pose",
        output="screen",

        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,

            {
                "pose_type": pose_type,
            },
            
            pose,
            
            {
                "trajectory_name":
                    trajectory_name,

                "num_candidates":
                    num_candidates,

                "minimum_required_clearance":
                    minimum_required_clearance,

                "weight_clearance":
                    weight_clearance,

                "weight_smoothness":
                    weight_smoothness,

                "weight_path_length":
                    weight_path_length,

                "weight_duration":
                    weight_duration,
            },
        ],
    )

    return [move_node]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pose",
                default_value="meca_demo",
                description=(
                    "Name of target pose in poses.yaml"
                ),
            ),

            DeclareLaunchArgument(
                "trajectory_name",
                default_value="latest_selected",
                description=(
                    "Name used when saving "
                    "selected trajectory"
                ),
            ),

            DeclareLaunchArgument(
                "num_candidates",
                default_value="10",
                description=(
                    "Number of trajectory "
                    "candidates to generate"
                ),
            ),

            DeclareLaunchArgument(
                "minimum_required_clearance",
                default_value="0.0",
                description=(
                    "Minimum allowed sampled "
                    "clearance in metres"
                ),
            ),

            DeclareLaunchArgument(
                "weight_clearance",
                default_value="0.40",
                description=(
                    "Weight assigned to clearance"
                ),
            ),

            DeclareLaunchArgument(
                "weight_smoothness",
                default_value="0.30",
                description=(
                    "Weight assigned to smoothness"
                ),
            ),

            DeclareLaunchArgument(
                "weight_path_length",
                default_value="0.20",
                description=(
                    "Weight assigned to "
                    "joint-space path length"
                ),
            ),

            DeclareLaunchArgument(
                "weight_duration",
                default_value="0.10",
                description=(
                    "Weight assigned to duration"
                ),
            ),

            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )