#include "meca500_tasks/trajectory_planner.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <yaml-cpp/yaml.h>

#include "meca500_tasks/trajectory_library.hpp"

namespace meca500_tasks
{

    struct Candidate
    {
        int number = -1;

        moveit::planning_interface::
            MoveGroupInterface::Plan plan;

        TrajectoryMetrics metrics;

        double score = 0.0;
    };

    PlanningResult planTrajectory(
        const std::shared_ptr<rclcpp::Node> &node,
        const PlanningRequest &request,
        const PlanningFeedbackCallback &feedback_callback)
    {
        PlanningResult result;

        result.trajectory_name =
            request.trajectory_name;

        const auto logger =
            node->get_logger();

        try
        {
            // =========================================================
            // VALIDATE REQUEST
            // =========================================================

            if (request.start_pose.empty())
            {
                throw std::runtime_error(
                    "Start pose name cannot be empty.");
            }

            if (request.target_pose.empty())
            {
                throw std::runtime_error(
                    "Target pose name cannot be empty.");
            }

            if (request.trajectory_name.empty())
            {
                throw std::runtime_error(
                    "Trajectory name cannot be empty.");
            }

            if (request.num_candidates <= 0)
            {
                throw std::runtime_error(
                    "num_candidates must be greater than 0.");
            }

            if (request.minimum_required_clearance < 0.0)
            {
                throw std::runtime_error(
                    "minimum_required_clearance cannot be negative.");
            }

            const double total_weight =
                request.weight_clearance +
                request.weight_smoothness +
                request.weight_path_length +
                request.weight_duration;

            if (std::abs(total_weight - 1.0) > 1e-6)
            {
                throw std::runtime_error(
                    "Trajectory weights must sum to 1.0.");
            }

            // =========================================================
            // LOAD SAVED POSES
            // =========================================================

            const std::filesystem::path poses_path =
                std::filesystem::current_path() / "src" / "meca500_tasks" / "config" / "poses.yaml";

            if (!std::filesystem::exists(poses_path))
            {
                throw std::runtime_error(
                    "Could not find poses.yaml.");
            }

            const YAML::Node poses =
                YAML::LoadFile(
                    poses_path.string());

            if (!poses[request.start_pose])
            {
                throw std::runtime_error(
                    "Unknown start pose '" +
                    request.start_pose +
                    "'.");
            }

            if (!poses[request.target_pose])
            {
                throw std::runtime_error(
                    "Unknown target pose '" +
                    request.target_pose +
                    "'.");
            }

            const YAML::Node start_pose =
                poses[request.start_pose];

            const YAML::Node target_pose =
                poses[request.target_pose];

            // =========================================================
            // LOAD START POSE
            // =========================================================

            if (!start_pose["type"] ||
                start_pose["type"].as<std::string>() != "joint")
            {
                throw std::runtime_error(
                    "Start pose '" +
                    request.start_pose +
                    "' must currently be a joint pose.");
            }

            if (!start_pose["joints"] ||
                start_pose["joints"].size() != 6)
            {
                throw std::runtime_error(
                    "Start pose '" +
                    request.start_pose +
                    "' must contain exactly 6 joint values.");
            }

            std::vector<double> start_joints;

            for (const auto &value :
                 start_pose["joints"])
            {
                start_joints.push_back(
                    value.as<double>());
            }

            // =========================================================
            // LOAD TARGET POSE
            // =========================================================

            if (!target_pose["type"])
            {
                throw std::runtime_error(
                    "Target pose has no type.");
            }

            const std::string target_type =
                target_pose["type"]
                    .as<std::string>();

            geometry_msgs::msg::Pose
                cartesian_target;

            std::vector<double>
                joint_target;

            if (target_type == "cartesian")
            {
                const std::vector<std::string>
                    required_fields = {
                        "x",
                        "y",
                        "z",
                        "qx",
                        "qy",
                        "qz",
                        "qw"};

                for (const auto &field :
                     required_fields)
                {
                    if (!target_pose[field])
                    {
                        throw std::runtime_error(
                            "Cartesian target pose is missing '" +
                            field +
                            "'.");
                    }
                }

                cartesian_target.position.x =
                    target_pose["x"].as<double>();

                cartesian_target.position.y =
                    target_pose["y"].as<double>();

                cartesian_target.position.z =
                    target_pose["z"].as<double>();

                cartesian_target.orientation.x =
                    target_pose["qx"].as<double>();

                cartesian_target.orientation.y =
                    target_pose["qy"].as<double>();

                cartesian_target.orientation.z =
                    target_pose["qz"].as<double>();

                cartesian_target.orientation.w =
                    target_pose["qw"].as<double>();
            }
            else if (target_type == "joint")
            {
                if (!target_pose["joints"] ||
                    target_pose["joints"].size() != 6)
                {
                    throw std::runtime_error(
                        "Joint target pose must contain exactly 6 values.");
                }

                for (const auto &value :
                     target_pose["joints"])
                {
                    joint_target.push_back(
                        value.as<double>());
                }
            }
            else
            {
                throw std::runtime_error(
                    "Unknown target pose type '" +
                    target_type +
                    "'.");
            }

            // =========================================================
            // MOVEIT SETUP
            // =========================================================

            using moveit::planning_interface::
                MoveGroupInterface;

            if (feedback_callback)
            {
                feedback_callback(
                    0,
                    request.num_candidates,
                    "Initialising MoveIt");
            }

            if (request.velocity_scaling <= 0.0 ||
                request.velocity_scaling > 1.0)
            {
                throw std::runtime_error(
                    "Velocity scaling must be between 0 and 1.");
            }

            if (request.acceleration_scaling <= 0.0 ||
                request.acceleration_scaling > 1.0)
            {
                throw std::runtime_error(
                    "Acceleration scaling must be between 0 and 1.");
            }

            MoveGroupInterface move_group_interface(
                node,
                "meca_arm");

            auto planning_scene_monitor =
                std::make_shared<
                    planning_scene_monitor::
                        PlanningSceneMonitor>(
                    node,
                    "robot_description");

            planning_scene_monitor
                ->startSceneMonitor();

            planning_scene_monitor
                ->startWorldGeometryMonitor();

            planning_scene_monitor
                ->startStateMonitor();

            planning_scene_monitor
                ->requestPlanningSceneState(
                    "/get_planning_scene");

            move_group_interface
                .setPoseReferenceFrame(
                    "world");

            move_group_interface
                .setEndEffectorLink(
                    "link_6");

            move_group_interface
                .setPlanningPipelineId(
                    "ompl");

            move_group_interface
                .setPlannerId(
                    "RRTConnectkConfigDefault");

            move_group_interface
                .setPlanningTime(
                    5.0);

            move_group_interface
                .setMaxVelocityScalingFactor(
                    request.velocity_scaling);

            move_group_interface
                .setMaxAccelerationScalingFactor(
                    request.acceleration_scaling);

            move_group_interface
                .setGoalPositionTolerance(
                    0.001);

            move_group_interface
                .setGoalOrientationTolerance(
                    0.01);

            // =========================================================
            // CREATE EXPLICIT START STATE
            // =========================================================

            const auto current_state =
                move_group_interface
                    .getCurrentState(10.0);

            if (!current_state)
            {
                throw std::runtime_error(
                    "No current robot state received.");
            }

            auto start_state =
                *current_state;

            const auto *joint_model_group =
                start_state.getJointModelGroup(
                    "meca_arm");

            if (!joint_model_group)
            {
                throw std::runtime_error(
                    "Could not find joint model group 'meca_arm'.");
            }

            start_state.setJointGroupPositions(
                joint_model_group,
                start_joints);

            start_state.update();

            move_group_interface
                .setStartState(
                    start_state);

            // =========================================================
            // SET TARGET
            // =========================================================

            if (target_type == "cartesian")
            {
                move_group_interface
                    .setPoseTarget(
                        cartesian_target);
            }
            else
            {
                move_group_interface
                    .setJointValueTarget(
                        joint_target);
            }

            // =========================================================
            // GENERATE CANDIDATES
            // =========================================================

            std::vector<Candidate>
                safe_candidates;

            int successful_plans = 0;

            RCLCPP_INFO(
                logger,
                "Generating %d trajectory candidates.",
                request.num_candidates);

            for (int i = 0;
                 i < request.num_candidates;
                 ++i)
            {
                if (feedback_callback)
                {
                    feedback_callback(
                        i + 1,
                        request.num_candidates,
                        "Planning candidate " +
                            std::to_string(i + 1));
                }

                MoveGroupInterface::Plan
                    candidate_plan;

                const bool planning_success =
                    static_cast<bool>(
                        move_group_interface.plan(
                            candidate_plan));

                if (!planning_success)
                {
                    RCLCPP_WARN(
                        logger,
                        "Candidate %d | PLANNING FAILED",
                        i + 1);

                    continue;
                }

                ++successful_plans;

                const auto &trajectory =
                    candidate_plan
                        .trajectory
                        .joint_trajectory;

                if (feedback_callback)
                {
                    feedback_callback(
                        i + 1,
                        request.num_candidates,
                        "Evaluating candidate " +
                            std::to_string(i + 1));
                }

                planning_scene_monitor::
                    LockedPlanningSceneRO scene(
                        planning_scene_monitor);

                const auto metrics =
                    evaluateTrajectory(
                        trajectory,
                        scene);

                const bool is_safe =
                    metrics.minimum_clearance >
                    request.minimum_required_clearance;

                RCLCPP_INFO(
                    logger,
                    "\n"
                    "Candidate %d\n"
                    "  Status       : %s\n"
                    "  Clearance    : %.1f mm\n"
                    "  Closest pair : %s <-> %s\n"
                    "  Smoothness   : %.6f\n"
                    "  Path length  : %.4f rad\n"
                    "  Duration     : %.3f s",
                    i + 1,
                    is_safe ? "SAFE" : "REJECTED",
                    metrics.minimum_clearance * 1000.0,
                    metrics.closest_object_a.c_str(),
                    metrics.closest_object_b.c_str(),
                    metrics.smoothness,
                    metrics.path_length,
                    metrics.duration);

                if (!is_safe)
                {
                    continue;
                }

                Candidate candidate;

                candidate.number =
                    i + 1;

                candidate.plan =
                    candidate_plan;

                candidate.metrics =
                    metrics;

                safe_candidates.push_back(
                    candidate);
            }

            // =========================================================
            // CHECK WHETHER ANY USABLE PLAN EXISTS
            // =========================================================

            if (safe_candidates.empty())
            {
                if (successful_plans == 0)
                {
                    result.message =
                        "All trajectory candidates failed to plan.";
                }
                else
                {
                    result.message =
                        "No trajectory candidate passed the clearance gate.";
                }

                result.success = false;

                return result;
            }

            // =========================================================
            // FIND METRIC RANGES
            // =========================================================

            double min_clearance =
                std::numeric_limits<double>::infinity();

            double max_clearance =
                -std::numeric_limits<double>::infinity();

            double min_smoothness =
                std::numeric_limits<double>::infinity();

            double max_smoothness =
                -std::numeric_limits<double>::infinity();

            double min_path_length =
                std::numeric_limits<double>::infinity();

            double max_path_length =
                -std::numeric_limits<double>::infinity();

            double min_duration =
                std::numeric_limits<double>::infinity();

            double max_duration =
                -std::numeric_limits<double>::infinity();

            for (const auto &candidate :
                 safe_candidates)
            {
                const auto &m =
                    candidate.metrics;

                min_clearance =
                    std::min(
                        min_clearance,
                        m.minimum_clearance);

                max_clearance =
                    std::max(
                        max_clearance,
                        m.minimum_clearance);

                min_smoothness =
                    std::min(
                        min_smoothness,
                        m.smoothness);

                max_smoothness =
                    std::max(
                        max_smoothness,
                        m.smoothness);

                min_path_length =
                    std::min(
                        min_path_length,
                        m.path_length);

                max_path_length =
                    std::max(
                        max_path_length,
                        m.path_length);

                min_duration =
                    std::min(
                        min_duration,
                        m.duration);

                max_duration =
                    std::max(
                        max_duration,
                        m.duration);
            }

            // =========================================================
            // NORMALISATION
            // =========================================================

            auto normalise =
                [](double value,
                   double min_value,
                   double max_value)
            {
                const double range =
                    max_value - min_value;

                if (std::abs(range) < 1e-9)
                {
                    return 0.0;
                }

                return (value - min_value) /
                       range;
            };

            const bool clearance_varies =
                std::abs(
                    max_clearance -
                    min_clearance) >= 1e-9;

            // =========================================================
            // CALCULATE WEIGHTED COST
            // =========================================================

            if (feedback_callback)
            {
                feedback_callback(
                    request.num_candidates,
                    request.num_candidates,
                    "Scoring candidates");
            }

            for (auto &candidate :
                 safe_candidates)
            {
                const auto &m =
                    candidate.metrics;

                const double clearance_normalised =
                    normalise(
                        m.minimum_clearance,
                        min_clearance,
                        max_clearance);

                const double smoothness_normalised =
                    normalise(
                        m.smoothness,
                        min_smoothness,
                        max_smoothness);

                const double path_length_normalised =
                    normalise(
                        m.path_length,
                        min_path_length,
                        max_path_length);

                const double duration_normalised =
                    normalise(
                        m.duration,
                        min_duration,
                        max_duration);

                const double clearance_penalty =
                    clearance_varies
                        ? 1.0 -
                              clearance_normalised
                        : 0.0;

                candidate.score =
                    request.weight_clearance *
                        clearance_penalty +

                    request.weight_smoothness *
                        smoothness_normalised +

                    request.weight_path_length *
                        path_length_normalised +

                    request.weight_duration *
                        duration_normalised;
            }

            // =========================================================
            // SELECT LOWEST COST
            // =========================================================

            const auto best_it =
                std::min_element(
                    safe_candidates.begin(),
                    safe_candidates.end(),
                    [](const Candidate &a,
                       const Candidate &b)
                    {
                        return a.score < b.score;
                    });

            const Candidate &best =
                *best_it;

            // =========================================================
            // SAVE SELECTED TRAJECTORY
            // =========================================================

            if (feedback_callback)
            {
                feedback_callback(
                    request.num_candidates,
                    request.num_candidates,
                    "Saving selected trajectory");
            }

            const bool saved =
                saveTrajectory(
                    request.trajectory_name,

                    best.plan
                        .trajectory
                        .joint_trajectory,

                    best.metrics,

                    request.start_pose,
                    request.target_pose,

                    request.num_candidates,

                    request.minimum_required_clearance,

                    request.weight_clearance,
                    request.weight_smoothness,
                    request.weight_path_length,
                    request.weight_duration);

            if (!saved)
            {
                result.success = false;

                result.message =
                    "Failed to save selected trajectory.";

                return result;
            }

            // =========================================================
            // RESULT
            // =========================================================

            result.success =
                true;

            result.message =
                "Trajectory planned and saved.";

            result.selected_candidate =
                best.number;

            result.weighted_cost =
                best.score;

            result.metrics =
                best.metrics;

            RCLCPP_INFO(
                logger,
                "\n"
                "==============================\n"
                "TRAJECTORY SELECTION SUMMARY\n"
                "==============================\n"
                "Selected candidate : %d\n"
                "Weighted cost      : %.4f\n"
                "Clearance          : %.1f mm\n"
                "Smoothness         : %.6f\n"
                "Path length        : %.4f rad\n"
                "Duration           : %.3f s\n"
                "Saved as           : %s\n"
                "==============================",
                result.selected_candidate,
                result.weighted_cost,
                result.metrics.minimum_clearance *
                    1000.0,
                result.metrics.smoothness,
                result.metrics.path_length,
                result.metrics.duration,
                request.trajectory_name.c_str());

            return result;
        }
        catch (const std::exception &error)
        {
            result.success =
                false;

            result.message =
                error.what();

            RCLCPP_ERROR(
                logger,
                "Planning failed: %s",
                error.what());

            return result;
        }
    }

} // namespace meca500_tasks
