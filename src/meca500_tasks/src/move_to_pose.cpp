#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include "meca500_tasks/trajectory_evaluator.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const node =
        std::make_shared<rclcpp::Node>(
            "meca_move_to_pose",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true));

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

        if (!move_group_interface
                 .getCurrentState(10.0))
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
            // TRAJECTORY SELECTION SETTINGS
            // -----------------------------------------------------

            constexpr int num_candidates = 10;

            // Currently only collision / penetration is rejected.
            // Replace with a validated physical margin later.
            constexpr double
                minimum_required_clearance = 0.0;

            // Treat sufficiently similar values as ties.
            constexpr double
                smoothness_epsilon = 1e-4;

            constexpr double
                path_length_epsilon = 1e-4;

            constexpr double
                duration_epsilon = 1e-3;

            MoveGroupInterface::Plan best_plan;

            meca500_tasks::TrajectoryMetrics
                best_metrics;

            best_metrics.smoothness =
                std::numeric_limits<double>::infinity();

            best_metrics.path_length =
                std::numeric_limits<double>::infinity();

            best_metrics.duration =
                std::numeric_limits<double>::infinity();

            int best_candidate = -1;

            int successful_plans = 0;
            int safe_plans = 0;

            RCLCPP_INFO(
                logger,
                "Generating %d trajectory candidates...",
                num_candidates);

            // -----------------------------------------------------
            // GENERATE AND EVALUATE CANDIDATES
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

                // One clean output block per candidate.
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

                // -------------------------------------------------
                // HIERARCHICAL OPTIMISATION
                //
                // 1. Smoothness
                // 2. Path length
                // 3. Duration
                //
                // Clearance is a hard constraint,
                // not a weighted cost.
                // -------------------------------------------------

                bool is_better = false;

                if (best_candidate == -1)
                {
                    is_better = true;
                }
                else if (
                    metrics.smoothness <
                    best_metrics.smoothness -
                        smoothness_epsilon)
                {
                    is_better = true;
                }
                else if (
                    std::abs(
                        metrics.smoothness -
                        best_metrics.smoothness) <= smoothness_epsilon)
                {
                    if (
                        metrics.path_length <
                        best_metrics.path_length -
                            path_length_epsilon)
                    {
                        is_better = true;
                    }
                    else if (
                        std::abs(
                            metrics.path_length -
                            best_metrics.path_length) <= path_length_epsilon)
                    {
                        if (
                            metrics.duration <
                            best_metrics.duration -
                                duration_epsilon)
                        {
                            is_better = true;
                        }
                    }
                }

                if (is_better)
                {
                    best_candidate = i + 1;
                    best_plan = candidate_plan;
                    best_metrics = metrics;
                }
            }

            // -----------------------------------------------------
            // FINAL RESULT
            // -----------------------------------------------------

            if (best_candidate != -1)
            {
                RCLCPP_INFO(
                    logger,
                    "\n"
                    "==============================\n"
                    "TRAJECTORY SELECTION SUMMARY\n"
                    "==============================\n"
                    "Planned successfully : %d/%d\n"
                    "Passed safety gate    : %d/%d\n"
                    "\n"
                    "Selected candidate     : %d\n"
                    "Clearance              : %.1f mm\n"
                    "Closest pair           : %s <-> %s\n"
                    "Smoothness             : %.6f\n"
                    "Path length            : %.4f rad\n"
                    "Duration               : %.3f s\n"
                    "==============================",
                    successful_plans,
                    num_candidates,
                    safe_plans,
                    successful_plans,
                    best_candidate,
                    best_metrics.minimum_clearance *
                        1000.0,
                    best_metrics.closest_object_a.c_str(),
                    best_metrics.closest_object_b.c_str(),
                    best_metrics.smoothness,
                    best_metrics.path_length,
                    best_metrics.duration);

                // -------------------------------------------------
                // EXECUTION
                // -------------------------------------------------

                const auto result =
                    move_group_interface.execute(
                        best_plan);

                if (
                    result ==
                    moveit::core::
                        MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(
                        logger,
                        "Selected trajectory executed successfully.");

                    exit_code = 0;
                }
                else
                {
                    RCLCPP_ERROR(
                        logger,
                        "Execution of selected trajectory failed.");
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