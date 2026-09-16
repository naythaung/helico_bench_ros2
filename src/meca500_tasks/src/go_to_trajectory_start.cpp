#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include "meca500_tasks/trajectory_library.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const node =
        std::make_shared<rclcpp::Node>(
            "meca_go_to_trajectory_start",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(
                    true));

    auto const logger =
        node->get_logger();

    const std::string trajectory_name =
        node->get_parameter("trajectory_name").as_string();

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
        using moveit::planning_interface::MoveGroupInterface;

        trajectory_msgs::msg::JointTrajectory trajectory;

        if (!meca500_tasks::loadTrajectory(
                trajectory_name,
                trajectory))
        {
            RCLCPP_ERROR(
                logger,
                "Could not load trajectory '%s'.",
                trajectory_name.c_str());

            throw std::runtime_error(
                "Trajectory loading failed");
        }

        if (trajectory.points.empty())
        {
            throw std::runtime_error(
                "Saved trajectory has no points");
        }

        const auto &start_positions =
            trajectory.points.front().positions;

        MoveGroupInterface move_group_interface(
            node,
            "meca_arm");

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
            .setStartStateToCurrentState();

        const bool target_set =
            move_group_interface
                .setJointValueTarget(
                    start_positions);

        if (!target_set)
        {
            throw std::runtime_error(
                "Could not set saved trajectory start as joint target");
        }

        MoveGroupInterface::Plan plan;

        const bool planning_success =
            static_cast<bool>(
                move_group_interface.plan(plan));

        if (!planning_success)
        {
            RCLCPP_ERROR(
                logger,
                "Could not plan to saved trajectory start.");

            throw std::runtime_error(
                "Planning failed");
        }

        RCLCPP_INFO(
            logger,
            "Planned route to start of '%s'.",
            trajectory_name.c_str());

        const auto result =
            move_group_interface.execute(
                plan);

        if (
            result ==
            moveit::core::
                MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                logger,
                "Reached saved trajectory start state.");

            exit_code = 0;
        }
        else
        {
            RCLCPP_ERROR(
                logger,
                "Execution to saved trajectory start failed.");
        }
    }
    catch (const std::exception &error)
    {
        RCLCPP_ERROR(
            logger,
            "%s",
            error.what());
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return exit_code;
}
