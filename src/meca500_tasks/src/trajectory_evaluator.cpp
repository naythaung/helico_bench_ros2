#include "meca500_tasks/trajectory_evaluator.hpp"

#include <cmath>

#include <limits>
#include <moveit/robot_state/robot_state.hpp>

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

        for (std::size_t i = 1; i < points.size() - 1; ++i)
        {
            for (std::size_t j = 0;
                 j < points[i].positions.size();
                 ++j)
            {
                const double second_difference =
                    points[i + 1].positions[j] - 2.0 * points[i].positions[j] + points[i - 1].positions[j];

                smoothness +=
                    second_difference * second_difference;
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

        return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
    }

    double calculateMinimumClearance(
        const trajectory_msgs::msg::JointTrajectory &trajectory,
        const planning_scene::PlanningSceneConstPtr &planning_scene)
    {
        if (!planning_scene || trajectory.points.empty())
            return 0.0;

        double minimum_clearance =
            std::numeric_limits<double>::infinity();

        moveit::core::RobotState state(
            planning_scene->getRobotModel());

        state = planning_scene->getCurrentState();

        const auto &acm =
            planning_scene->getAllowedCollisionMatrix();

        for (const auto &point : trajectory.points)
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

            const double clearance =
                planning_scene->distanceToCollision(
                    state,
                    acm);

            if (clearance < minimum_clearance)
            {
                minimum_clearance = clearance;
            }
        }

        return minimum_clearance;
    }

} // namespace meca500_tasks