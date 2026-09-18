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

    start_pose_name = LaunchConfiguration(
        "start_pose"
    ).perform(context)

    target_pose_name = LaunchConfiguration(
        "target_pose"
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
    # 2. Load saved poses
    # ---------------------------------------------------------

    poses_path = os.path.join(
        get_package_share_directory("meca500_tasks"),
        "config",
        "poses.yaml",
    )

    with open(poses_path, "r") as file:
        poses = yaml.safe_load(file)

    if start_pose_name not in poses:
        raise RuntimeError(
            f"Unknown start pose '{start_pose_name}'. "
            f"Available poses: {list(poses.keys())}"
        )

    if target_pose_name not in poses:
        raise RuntimeError(
            f"Unknown target pose '{target_pose_name}'. "
            f"Available poses: {list(poses.keys())}"
        )

    start_pose = poses[start_pose_name]
    target_pose = poses[target_pose_name]

    # Start state is deliberately joint-space for now.
    if start_pose.get("type") != "joint":
        raise RuntimeError(
            f"Start pose '{start_pose_name}' must currently "
            "be a joint pose."
        )

    if len(start_pose.get("joints", [])) != 6:
        raise RuntimeError(
            f"Start pose '{start_pose_name}' must contain "
            "exactly 6 joint values."
        )

    target_pose_type = target_pose.get("type")

    if target_pose_type not in ["cartesian", "joint"]:
        raise RuntimeError(
            f"Target pose '{target_pose_name}' has invalid type "
            f"'{target_pose_type}'."
        )

    print(
        f"Planning: {start_pose_name} -> {target_pose_name}"
    )

    print(
        f"Target pose type: {target_pose_type}"
    )

    print("Trajectory planning settings:")
    print(f"  Candidates        : {num_candidates}")

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
    # 3. Robot model
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
    # 4. Target parameters
    # ---------------------------------------------------------

    target_parameters = {
        "pose_type": target_pose_type,
    }

    if target_pose_type == "cartesian":
        target_parameters.update(
            {
                "x": float(target_pose["x"]),
                "y": float(target_pose["y"]),
                "z": float(target_pose["z"]),
                "qx": float(target_pose["qx"]),
                "qy": float(target_pose["qy"]),
                "qz": float(target_pose["qz"]),
                "qw": float(target_pose["qw"]),
            }
        )

    else:
        target_parameters["joints"] = [
            float(value)
            for value in target_pose["joints"]
        ]

    # ---------------------------------------------------------
    # 5. Start planner node
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

            target_parameters,

            {
                "start_pose_name":
                    start_pose_name,

                "target_pose_name":
                    target_pose_name,

                "start_joints": [
                    float(value)
                    for value in start_pose["joints"]
                ],

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
                "start_pose",
                default_value="meca_zero",
                description="Named joint-space start pose",
            ),

            DeclareLaunchArgument(
                "target_pose",
                default_value="meca_demo",
                description="Named target pose",
            ),

            DeclareLaunchArgument(
                "trajectory_name",
                default_value="latest_selected",
                description="Saved trajectory name",
            ),

            DeclareLaunchArgument(
                "num_candidates",
                default_value="10",
            ),

            DeclareLaunchArgument(
                "minimum_required_clearance",
                default_value="0.0",
            ),

            DeclareLaunchArgument(
                "weight_clearance",
                default_value="0.40",
            ),

            DeclareLaunchArgument(
                "weight_smoothness",
                default_value="0.30",
            ),

            DeclareLaunchArgument(
                "weight_path_length",
                default_value="0.20",
            ),

            DeclareLaunchArgument(
                "weight_duration",
                default_value="0.10",
            ),

            OpaqueFunction(
                function=launch_setup
            ),
        ]
    )