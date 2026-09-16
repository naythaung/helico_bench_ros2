#pragma once

#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>

namespace meca500_tasks
{

struct TrajectoryMetrics
{
    double path_length = 0.0;
    double smoothness = 0.0;
    double duration = 0.0;
    double minimum_clearance = 0.0;

    std::string closest_object_a;
    std::string closest_object_b;
};

double calculatePathLength(
    const trajectory_msgs::msg::JointTrajectory &trajectory);

double calculateSmoothness(
    const trajectory_msgs::msg::JointTrajectory &trajectory);

double calculateDuration(
    const trajectory_msgs::msg::JointTrajectory &trajectory);

TrajectoryMetrics evaluateTrajectory(
    const trajectory_msgs::msg::JointTrajectory &trajectory,
    const planning_scene::PlanningSceneConstPtr &planning_scene);

} // namespace meca500_tasks