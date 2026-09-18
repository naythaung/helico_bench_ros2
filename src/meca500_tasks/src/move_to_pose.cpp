#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include "meca500_tasks/trajectory_evaluator.hpp"
#include "meca500_tasks/trajectory_library.hpp"

struct Candidate
{
    int number = -1;

    moveit::planning_interface::
        MoveGroupInterface::Plan plan;

    meca500_tasks::TrajectoryMetrics metrics;

    double score = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const node =
        std::make_shared<rclcpp::Node>(
            "meca_move_to_pose",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true));

    // Name used when saving the selected trajectory.
    const std::string trajectory_name =
        node->get_parameter("trajectory_name").as_string();

    auto const logger =
        node->get_logger();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner(
        [&executor]()
        {
            executor.spin();
        });

    int exit_code = 1;

    try
    {
        using moveit::planning_interface::
            MoveGroupInterface;

        // ---------------------------------------------------------
        // MOVEIT SETUP
        // ---------------------------------------------------------

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
            .setPoseReferenceFrame("world");

        move_group_interface
            .setEndEffectorLink("link_6");

        move_group_interface
            .setPlanningPipelineId("ompl");

        move_group_interface
            .setPlannerId(
                "RRTConnectkConfigDefault");

        move_group_interface
            .setPlanningTime(5.0);

        move_group_interface
            .setMaxVelocityScalingFactor(0.2);

        move_group_interface
            .setMaxAccelerationScalingFactor(0.2);

        move_group_interface
            .setGoalPositionTolerance(0.001);

        move_group_interface
            .setGoalOrientationTolerance(0.01);

        // ---------------------------------------------------------
        // TARGET POSE
        // ---------------------------------------------------------

        geometry_msgs::msg::Pose target_pose;

        target_pose.position.x =
            node->get_parameter("x").as_double();

        target_pose.position.y =
            node->get_parameter("y").as_double();

        target_pose.position.z =
            node->get_parameter("z").as_double();

        target_pose.orientation.x =
            node->get_parameter("qx").as_double();

        target_pose.orientation.y =
            node->get_parameter("qy").as_double();

        target_pose.orientation.z =
            node->get_parameter("qz").as_double();

        target_pose.orientation.w =
            node->get_parameter("qw").as_double();

        if (!move_group_interface.getCurrentState(10.0))
        {
            RCLCPP_ERROR(
                logger,
                "No current robot state received.");
        }
        else
        {
            move_group_interface
                .setStartStateToCurrentState();

            move_group_interface
                .setPoseTarget(target_pose);

            // -----------------------------------------------------
            // TRAJECTORY SETTINGS
            // -----------------------------------------------------

            const int num_candidates =
                node->get_parameter("num_candidates").as_int();

            const double minimum_required_clearance =
                node->get_parameter(
                        "minimum_required_clearance")
                    .as_double();

            const double weight_clearance =
                node->get_parameter(
                        "weight_clearance")
                    .as_double();

            const double weight_smoothness =
                node->get_parameter(
                        "weight_smoothness")
                    .as_double();

            const double weight_path_length =
                node->get_parameter(
                        "weight_path_length")
                    .as_double();

            const double weight_duration =
                node->get_parameter(
                        "weight_duration")
                    .as_double();

            if (num_candidates <= 0)
            {
                throw std::runtime_error(
                    "num_candidates must be greater than 0.");
            }

            const double total_weight =
                weight_clearance +
                weight_smoothness +
                weight_path_length +
                weight_duration;

            if (std::abs(total_weight - 1.0) > 1e-6)
            {
                throw std::runtime_error(
                    "Trajectory weights must sum to 1.0.");
            }

            if (minimum_required_clearance < 0.0)
            {
                throw std::runtime_error(
                    "minimum_required_clearance cannot be negative.");
            }

            // -----------------------------------------------------
            // CANDIDATE STORAGE / COUNTERS
            // -----------------------------------------------------

            int successful_plans = 0;
            int safe_plans = 0;

            std::vector<Candidate> safe_candidates;

            RCLCPP_INFO(
                logger,
                "Generating %d trajectory candidates...",
                num_candidates);

            // -----------------------------------------------------
            // GENERATE + EVALUATE ALL CANDIDATES
            // -----------------------------------------------------

            for (int i = 0;
                 i < num_candidates;
                 ++i)
            {
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

                planning_scene_monitor::
                    LockedPlanningSceneRO scene(
                        planning_scene_monitor);

                const auto metrics =
                    meca500_tasks::
                        evaluateTrajectory(
                            trajectory,
                            scene);

                const bool is_safe =
                    metrics.minimum_clearance >
                    minimum_required_clearance;

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
                    metrics.minimum_clearance *
                        1000.0,
                    metrics.closest_object_a.c_str(),
                    metrics.closest_object_b.c_str(),
                    metrics.smoothness,
                    metrics.path_length,
                    metrics.duration);

                // -------------------------------------------------
                // HARD SAFETY GATE
                // -------------------------------------------------

                if (!is_safe)
                {
                    continue;
                }

                ++safe_plans;

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

            // -----------------------------------------------------
            // WEIGHTED TRAJECTORY SELECTION
            // -----------------------------------------------------

            if (!safe_candidates.empty())
            {
                // Find minimum and maximum values across
                // all safe candidates.

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

                // Min-max normalisation:
                //
                // x_hat =
                // (x - x_min) /
                // (x_max - x_min)
                //
                // This converts each metric to 0 -> 1.

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

                // -------------------------------------------------
                // CALCULATE WEIGHTED COST
                //
                // J =
                // 0.40(1 - C_hat)
                // + 0.30 S_hat
                // + 0.20 L_hat
                // + 0.10 T_hat
                //
                // Lower J is better.
                // -------------------------------------------------

                for (auto &candidate :
                     safe_candidates)
                {
                    const auto &m =
                        candidate.metrics;

                    const double
                        clearance_normalised =
                            normalise(
                                m.minimum_clearance,
                                min_clearance,
                                max_clearance);

                    const double
                        smoothness_normalised =
                            normalise(
                                m.smoothness,
                                min_smoothness,
                                max_smoothness);

                    const double
                        path_length_normalised =
                            normalise(
                                m.path_length,
                                min_path_length,
                                max_path_length);

                    const double
                        duration_normalised =
                            normalise(
                                m.duration,
                                min_duration,
                                max_duration);

                    // More clearance is better,
                    // so invert the normalised value.
                    //
                    // If all candidates have exactly the
                    // same clearance, clearance cannot
                    // distinguish them and contributes zero.

                    const double
                        clearance_penalty =
                            clearance_varies
                                ? 1.0 -
                                      clearance_normalised
                                : 0.0;

                    candidate.score =
                        weight_clearance *
                            clearance_penalty +
                        weight_smoothness *
                            smoothness_normalised +
                        weight_path_length *
                            path_length_normalised +
                        weight_duration *
                            duration_normalised;
                }

                // -------------------------------------------------
                // PRINT WEIGHTED RESULTS
                // -------------------------------------------------

                RCLCPP_INFO(
                    logger,
                    "\n"
                    "==============================\n"
                    "WEIGHTED CANDIDATE SCORES\n"
                    "==============================");

                for (const auto &candidate :
                     safe_candidates)
                {
                    RCLCPP_INFO(
                        logger,
                        "Candidate %d | Cost %.4f",
                        candidate.number,
                        candidate.score);
                }

                // -------------------------------------------------
                // SELECT LOWEST COST
                // -------------------------------------------------

                const auto best_it =
                    std::min_element(
                        safe_candidates.begin(),
                        safe_candidates.end(),
                        [](const Candidate &a,
                           const Candidate &b)
                        {
                            return a.score <
                                   b.score;
                        });

                const int best_candidate =
                    best_it->number;

                const auto best_plan =
                    best_it->plan;

                const auto best_metrics =
                    best_it->metrics;

                const double best_score =
                    best_it->score;

                // -------------------------------------------------
                // FINAL RESULT
                // -------------------------------------------------

                RCLCPP_INFO(
                    logger,
                    "\n"
                    "==============================\n"
                    "TRAJECTORY SELECTION SUMMARY\n"
                    "==============================\n"
                    "Selection method       : Weighted multi-objective\n"
                    "Weights                : C=%.2f S=%.2f L=%.2f T=%.2f\n"
                    "\n"
                    "Planned successfully   : %d/%d\n"
                    "Passed safety gate      : %d/%d\n"
                    "\n"
                    "Selected candidate      : %d\n"
                    "Weighted cost           : %.4f\n"
                    "Clearance               : %.1f mm\n"
                    "Closest pair            : %s <-> %s\n"
                    "Smoothness              : %.6f\n"
                    "Path length             : %.4f rad\n"
                    "Duration                : %.3f s\n"
                    "==============================",

                    weight_clearance,
                    weight_smoothness,
                    weight_path_length,
                    weight_duration,

                    successful_plans,
                    num_candidates,
                    safe_plans,
                    successful_plans,
                    best_candidate,
                    best_score,
                    best_metrics.minimum_clearance * 1000.0,
                    best_metrics.closest_object_a.c_str(),
                    best_metrics.closest_object_b.c_str(),
                    best_metrics.smoothness,
                    best_metrics.path_length,
                    best_metrics.duration);

                // -------------------------------------------------
                // SAVE SELECTED TRAJECTORY
                //
                // Planning and execution are deliberately separated.
                // This node only plans, evaluates, selects and saves.
                // The saved trajectory can then be reviewed before
                // go_to_trajectory_start + replay_trajectory.
                // -------------------------------------------------

                const bool saved =
                    meca500_tasks::
                        saveTrajectory(
                            trajectory_name,
                            best_plan
                                .trajectory
                                .joint_trajectory,
                            best_metrics);

                if (saved)
                {
                    RCLCPP_INFO(
                        logger,
                        "Selected trajectory saved as '%s'.",
                        trajectory_name.c_str());

                    RCLCPP_INFO(
                        logger,
                        "Trajectory was NOT executed automatically.");

                    exit_code = 0;
                }
                else
                {
                    RCLCPP_ERROR(
                        logger,
                        "Failed to save selected trajectory.");
                }
            }
            else if (successful_plans > 0)
            {
                RCLCPP_ERROR(
                    logger,
                    "Planning succeeded, but no candidate "
                    "passed the clearance safety gate.");
            }
            else
            {
                RCLCPP_ERROR(
                    logger,
                    "All trajectory candidates failed "
                    "to plan.");
            }
        }
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(
            logger,
            "Task failed: %s",
            error.what());
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return exit_code;
}