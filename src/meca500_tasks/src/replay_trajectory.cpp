#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include "meca500_tasks/trajectory_library.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto const node =
        std::make_shared<rclcpp::Node>(
            "meca_replay_trajectory",
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

        // ---------------------------------------------------------
        // LOAD SAVED TRAJECTORY
        // ---------------------------------------------------------

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

        RCLCPP_INFO(
            logger,
            "Loaded trajectory '%s' with %zu points.",
            trajectory_name.c_str(),
            trajectory.points.size());

        // ---------------------------------------------------------
        // MOVEIT
        // ---------------------------------------------------------

        MoveGroupInterface move_group_interface(
            node,
            "meca_arm");

        const auto current_state =
            move_group_interface.getCurrentState(10.0);

        if (!current_state)
        {
            throw std::runtime_error(
                "No current robot state received");
        }

        const auto *joint_model_group =
            current_state->getJointModelGroup(
                "meca_arm");

        if (!joint_model_group)
        {
            throw std::runtime_error(
                "Could not find joint group meca_arm");
        }

        std::vector<double> current_joints;

        current_state->copyJointGroupPositions(
            joint_model_group,
            current_joints);

        const auto &expected_start =
            trajectory.points.front().positions;

        if (current_joints.size() !=
            expected_start.size())
        {
            throw std::runtime_error(
                "Joint count does not match saved trajectory");
        }

        // ---------------------------------------------------------
        // START STATE CHECK
        // ---------------------------------------------------------

        constexpr double
            start_tolerance = 0.05; // rad

        bool start_matches = true;

        RCLCPP_INFO(
            logger,
            "Checking trajectory start state...");

        for (std::size_t i = 0;
             i < current_joints.size();
             ++i)
        {
            const double error =
                std::abs(
                    current_joints[i] -
                    expected_start[i]);

            RCLCPP_INFO(
                logger,
                "  Joint %zu | current %.4f | expected %.4f | error %.4f rad",
                i + 1,
                current_joints[i],
                expected_start[i],
                error);

            if (error > start_tolerance)
            {
                start_matches = false;
            }
        }

        if (!start_matches)
        {
            RCLCPP_ERROR(
                logger,
                "Robot is not at the saved trajectory start state.");

            RCLCPP_ERROR(
                logger,
                "Replay refused.");

            throw std::runtime_error(
                "Start state mismatch");
        }

        RCLCPP_INFO(
            logger,
            "Start state valid.");

        // ---------------------------------------------------------
        // EXECUTE SAVED TRAJECTORY
        // ---------------------------------------------------------

        MoveGroupInterface::Plan replay_plan;

        replay_plan.trajectory.joint_trajectory =
            trajectory;

        RCLCPP_INFO(
            logger,
            "Executing saved trajectory '%s'...",
            trajectory_name.c_str());

        const auto result =
            move_group_interface.execute(
                replay_plan);

        if (
            result ==
            moveit::core::
                MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(
                logger,
                "Saved trajectory executed successfully.");

            exit_code = 0;
        }
        else
        {
            RCLCPP_ERROR(
                logger,
                "Saved trajectory execution failed.");
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
