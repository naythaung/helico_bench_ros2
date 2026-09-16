#include "meca500_tasks/trajectory_evaluator.hpp"

#include <cmath>
#include <limits>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/collision_detection/collision_env.hpp>

namespace meca500_tasks
{

double calculatePathLength(
    const trajectory_msgs::msg::JointTrajectory &trajectory)
{
    const auto &points = trajectory.points;

    if (points.size() < 2)
        return 0.0;

    double total_length = 0.0;

    for (std::size_t i = 1; i < points.size(); ++i)
    {
        double segment_squared = 0.0;

        for (std::size_t j = 0;
             j < points[i].positions.size();
             ++j)
        {
            const double dq =
                points[i].positions[j] -
                points[i - 1].positions[j];

            segment_squared += dq * dq;
        }

        total_length += std::sqrt(segment_squared);
    }

    return total_length;
}

double calculateSmoothness(
    const trajectory_msgs::msg::JointTrajectory &trajectory)
{
    const auto &points = trajectory.points;

    if (points.size() < 3)
        return 0.0;

    double smoothness = 0.0;

    for (std::size_t i = 1;
         i < points.size() - 1;
         ++i)
    {
        for (std::size_t j = 0;
             j < points[i].positions.size();
             ++j)
        {
            const double second_difference =
                points[i + 1].positions[j]
                - 2.0 * points[i].positions[j]
                + points[i - 1].positions[j];

            smoothness +=
                second_difference *
                second_difference;
        }
    }

    return smoothness;
}

double calculateDuration(
    const trajectory_msgs::msg::JointTrajectory &trajectory)
{
    if (trajectory.points.empty())
        return 0.0;

    const auto &time =
        trajectory.points.back().time_from_start;

    return static_cast<double>(time.sec)
        + static_cast<double>(time.nanosec) * 1e-9;
}

TrajectoryMetrics evaluateTrajectory(
    const trajectory_msgs::msg::JointTrajectory &trajectory,
    const planning_scene::PlanningSceneConstPtr &planning_scene)
{
    TrajectoryMetrics metrics;

    metrics.path_length =
        calculatePathLength(trajectory);

    metrics.smoothness =
        calculateSmoothness(trajectory);

    metrics.duration =
        calculateDuration(trajectory);

    if (!planning_scene ||
        trajectory.points.empty())
    {
        return metrics;
    }

    metrics.minimum_clearance =
        std::numeric_limits<double>::infinity();

    moveit::core::RobotState state =
        planning_scene->getCurrentState();

    const auto &acm =
        planning_scene->getAllowedCollisionMatrix();

    // Clearance sampling is used for ranking / diagnostics.
    // MoveIt remains responsible for full collision checking
    // during planning.
    constexpr std::size_t sample_stride = 5;

    auto evaluate_point =
        [&](const trajectory_msgs::msg::JointTrajectoryPoint &point)
    {
        for (std::size_t j = 0;
             j < trajectory.joint_names.size();
             ++j)
        {
            state.setVariablePosition(
                trajectory.joint_names[j],
                point.positions[j]);
        }

        state.update();

        collision_detection::DistanceRequest request;
        collision_detection::DistanceResult result;

        request.type =
            collision_detection::DistanceRequestType::SINGLE;

        request.enable_nearest_points = false;
        request.enable_signed_distance = true;
        request.acm = &acm;

        planning_scene
            ->getCollisionEnv()
            ->distanceRobot(
                request,
                result,
                state);

        const double clearance =
            result.minimum_distance.distance;

        if (clearance <
            metrics.minimum_clearance)
        {
            metrics.minimum_clearance =
                clearance;

            metrics.closest_object_a =
                result.minimum_distance.link_names[0];

            metrics.closest_object_b =
                result.minimum_distance.link_names[1];
        }
    };

    // Sample every fifth waypoint.
    for (std::size_t i = 0;
         i < trajectory.points.size();
         i += sample_stride)
    {
        evaluate_point(
            trajectory.points[i]);
    }

    // Always check the final trajectory point.
    const std::size_t final_index =
        trajectory.points.size() - 1;

    if (final_index % sample_stride != 0)
    {
        evaluate_point(
            trajectory.points.back());
    }

    return metrics;
}

} // namespace meca500_tasks