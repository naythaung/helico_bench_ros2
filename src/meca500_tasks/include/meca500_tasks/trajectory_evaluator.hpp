#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>

namespace meca500_tasks
{

double calculatePathLength(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

double calculateSmoothness(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

double calculateDuration(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

double calculateMinimumClearance(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const planning_scene::PlanningSceneConstPtr& planning_scene);

}  // namespace meca500_tasks