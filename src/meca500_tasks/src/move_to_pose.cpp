#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "meca500_tasks/trajectory_evaluator.hpp"
#include <limits>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

int main(int argc, char *argv[])
{
    // 1. Initialise ROS and create our node.
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "meca_move_to_pose",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    auto const logger = node->get_logger();

    // Process incoming ROS messages while we wait for MoveIt.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]()
                        { executor.spin(); });

    int exit_code = 1;

    try
    {
        // 2. Connect to our existing MoveIt planning group.
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, "meca_arm");

        auto planning_scene_monitor =
            std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
                node,
                "robot_description");

        planning_scene_monitor->startSceneMonitor();
        planning_scene_monitor->startWorldGeometryMonitor();
        planning_scene_monitor->startStateMonitor();

        move_group_interface.setPoseReferenceFrame("world");
        move_group_interface.setEndEffectorLink("link_6");

        move_group_interface.setPlanningPipelineId("ompl");
        move_group_interface.setPlannerId("RRTConnectkConfigDefault");
        move_group_interface.setPlanningTime(5.0);

        move_group_interface.setMaxVelocityScalingFactor(0.2);
        move_group_interface.setMaxAccelerationScalingFactor(0.2);

        // Our captured target was rounded, so allow small tolerances.
        move_group_interface.setGoalPositionTolerance(0.001);
        move_group_interface.setGoalOrientationTolerance(0.01);

        // 3. Read the target pose from ROS parameters.
        double x = node->get_parameter("x").as_double();
        double y = node->get_parameter("y").as_double();
        double z = node->get_parameter("z").as_double();

        double qx = node->get_parameter("qx").as_double();
        double qy = node->get_parameter("qy").as_double();
        double qz = node->get_parameter("qz").as_double();
        double qw = node->get_parameter("qw").as_double();

        geometry_msgs::msg::Pose target_pose;

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;

        if (!move_group_interface.getCurrentState(10.0))
        {
            RCLCPP_ERROR(logger, "No current robot state received.");
        }
        else
        {
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(target_pose);

            // 4. Generate several candidate trajectories from the same start state.
            constexpr int num_candidates = 10;

            MoveGroupInterface::Plan best_plan;

            double best_path_length =
                std::numeric_limits<double>::infinity();

            int best_candidate = -1;
            int successful_plans = 0;

            for (int i = 0; i < num_candidates; ++i)
            {
                MoveGroupInterface::Plan candidate_plan;

                const bool success = static_cast<bool>(
                    move_group_interface.plan(candidate_plan));

                if (!success)
                {
                    RCLCPP_WARN(
                        logger,
                        "Candidate %d: planning failed.",
                        i + 1);

                    continue;
                }

                ++successful_plans;

                const auto &trajectory =
                    candidate_plan.trajectory.joint_trajectory;

                const double path_length =
                    meca500_tasks::calculatePathLength(trajectory);

                const double smoothness =
                    meca500_tasks::calculateSmoothness(trajectory);

                const double duration =
                    meca500_tasks::calculateDuration(trajectory);

                planning_scene_monitor::LockedPlanningSceneRO scene(
                    planning_scene_monitor);

                const double minimum_clearance =
                    meca500_tasks::calculateMinimumClearance(
                        trajectory,
                        scene);

                RCLCPP_INFO(
                    logger,
                    "Candidate %d: length = %.4f rad | smoothness = %.6f | duration = %.3f s | clearance = %.4f m",
                    i + 1,
                    path_length,
                    smoothness,
                    duration,
                    minimum_clearance);

                // Keep this candidate if it is shorter than the current best.
                if (path_length < best_path_length)
                {
                    best_path_length = path_length;
                    best_plan = candidate_plan;
                    best_candidate = i + 1;
                }
            }

            // 5. Execute only the best candidate.
            if (best_candidate != -1)
            {
                RCLCPP_INFO(
                    logger,
                    "%d/%d candidates succeeded.",
                    successful_plans,
                    num_candidates);

                RCLCPP_INFO(
                    logger,
                    "Selected candidate %d with path length %.4f rad.",
                    best_candidate,
                    best_path_length);

                const auto result =
                    move_group_interface.execute(best_plan);

                if (result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(
                        logger,
                        "Best trajectory executed successfully.");

                    exit_code = 0;
                }
                else
                {
                    RCLCPP_ERROR(
                        logger,
                        "Execution of best trajectory failed.");
                }
            }
            else
            {
                RCLCPP_ERROR(
                    logger,
                    "All candidate trajectories failed to plan.");
            }
        }
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(logger, "Task failed: %s", error.what());
    }

    // 6. Stop message processing and shut down.
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return exit_code;
}