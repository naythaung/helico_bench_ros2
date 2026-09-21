#pragma once

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "meca500_tasks/trajectory_evaluator.hpp"

namespace meca500_tasks
{

    struct PlanningRequest
    {
        std::string start_pose;
        std::string target_pose;
        std::string trajectory_name;

        int num_candidates = 10;

        double velocity_scaling = 0.2;
        double acceleration_scaling = 0.2;

        double minimum_required_clearance = 0.0;

        double weight_clearance = 0.4;
        double weight_smoothness = 0.3;
        double weight_path_length = 0.2;
        double weight_duration = 0.1;
    };

    struct PlanningResult
    {
        bool success = false;

        std::string message;
        std::string trajectory_name;

        int selected_candidate = -1;

        double weighted_cost = 0.0;

        TrajectoryMetrics metrics;
    };

    using PlanningFeedbackCallback =
        std::function<void(
            int current_candidate,
            int total_candidates,
            const std::string &status)>;

    PlanningResult planTrajectory(
        const std::shared_ptr<rclcpp::Node> &node,
        const PlanningRequest &request,
        const PlanningFeedbackCallback &feedback_callback = nullptr);

} // namespace meca500_tasks
