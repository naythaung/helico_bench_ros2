#include "meca500_tasks/trajectory_evaluator.hpp"

#include <cmath>
#include <limits>
#include <unordered_set>

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
                    points[i + 1].positions[j] - 2.0 * points[i].positions[j] + points[i - 1].positions[j];

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

        return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
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

        collision_detection::AllowedCollisionMatrix filtered_acm =
            planning_scene->getAllowedCollisionMatrix();

        const std::unordered_set<std::string> moving_links = {
            "link_1",
            "link_2",
            "link_3",
            "link_4",
            "link_5",
            "link_6",
            "tools_mount",
            "laser_sensor_mount",
            "laser_sensor",
            "force_sensor",
            "force_poker",
            "april_tag_mount"};

        auto is_moving =
            [&moving_links](const std::string &name)
        {
            return moving_links.count(name) > 0;
        };

        // Clearance sampling is used for ranking / diagnostics.
        // MoveIt remains responsible for full collision checking
        // during planning.
        constexpr std::size_t sample_stride = 2;

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
            request.acm = &filtered_acm;

            // ---------------------------------------------------------
            // Find nearest RELEVANT pair
            //
            // Ignore static-static pairs because their separation cannot
            // change as the robot moves and therefore tells us nothing
            // about trajectory quality.
            // ---------------------------------------------------------

            constexpr int max_pair_searches = 50;

            for (int attempt = 0;
                 attempt < max_pair_searches;
                 ++attempt)
            {
                result.clear();

                planning_scene
                    ->getCollisionEnv()
                    ->distanceRobot(
                        request,
                        result,
                        state);

                const auto &distance =
                    result.minimum_distance;

                const std::string object_a =
                    distance.link_names[0];

                const std::string object_b =
                    distance.link_names[1];

                const bool relevant =
                    is_moving(object_a) ||
                    is_moving(object_b);

                if (relevant)
                {
                    const double clearance =
                        distance.distance;

                    if (clearance <
                        metrics.minimum_clearance)
                    {
                        metrics.minimum_clearance =
                            clearance;

                        metrics.closest_object_a =
                            object_a;

                        metrics.closest_object_b =
                            object_b;
                    }

                    break;
                }

                // This pair is static-static.
                // Exclude it from the clearance ranking calculation.
                filtered_acm.setEntry(
                    object_a,
                    object_b,
                    true);
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