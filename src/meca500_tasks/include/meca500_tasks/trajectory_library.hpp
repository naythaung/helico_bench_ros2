#pragma once

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "meca500_tasks/trajectory_evaluator.hpp"

namespace meca500_tasks
{

     inline bool saveTrajectory(
         const std::string &name,
         const trajectory_msgs::msg::JointTrajectory &trajectory,
         const TrajectoryMetrics &metrics,
         const std::string &start_pose_name,
         const std::string &target_pose_name,
         int num_candidates,
         double minimum_required_clearance,
         double weight_clearance,
         double weight_smoothness,
         double weight_path_length,
         double weight_duration)
     {
          const std::filesystem::path directory =
              std::filesystem::current_path() / "src" / "meca500_tasks" / "trajectories";

          std::filesystem::create_directories(directory);

          const std::filesystem::path filepath =
              directory / (name + ".yaml");

          std::ofstream file(filepath);

          if (!file.is_open())
               return false;

          file << std::setprecision(15);

          // ---------------------------------------------------------
          // METADATA
          // ---------------------------------------------------------

          file << "name: " << name << "\n";

          file << "start_pose: "
               << start_pose_name << "\n";

          file << "target_pose: "
               << target_pose_name << "\n";

          file << "planning:\n";

          file << "  num_candidates: "
               << num_candidates << "\n";

          file << "  minimum_required_clearance: "
               << minimum_required_clearance << "\n";

          file << "  weight_clearance: "
               << weight_clearance << "\n";

          file << "  weight_smoothness: "
               << weight_smoothness << "\n";

          file << "  weight_path_length: "
               << weight_path_length << "\n";

          file << "  weight_duration: "
               << weight_duration << "\n";

          file << "metrics:\n";

          file << "  minimum_clearance: "
               << metrics.minimum_clearance << "\n";

          file << "  closest_object_a: "
               << metrics.closest_object_a << "\n";

          file << "  closest_object_b: "
               << metrics.closest_object_b << "\n";

          file << "  smoothness: "
               << metrics.smoothness << "\n";

          file << "  path_length: "
               << metrics.path_length << "\n";

          file << "  duration: "
               << metrics.duration << "\n";

          // ---------------------------------------------------------
          // JOINT NAMES
          // ---------------------------------------------------------

          file << "joint_names:\n";

          for (const auto &joint_name : trajectory.joint_names)
          {
               file << "  - " << joint_name << "\n";
          }

          // ---------------------------------------------------------
          // TRAJECTORY POINTS
          // ---------------------------------------------------------

          file << "points:\n";

          for (const auto &point : trajectory.points)
          {
               file << "  - positions: [";

               for (std::size_t i = 0;
                    i < point.positions.size();
                    ++i)
               {
                    file << point.positions[i];

                    if (i + 1 < point.positions.size())
                         file << ", ";
               }

               file << "]\n";

               file << "    velocities: [";

               for (std::size_t i = 0;
                    i < point.velocities.size();
                    ++i)
               {
                    file << point.velocities[i];

                    if (i + 1 < point.velocities.size())
                         file << ", ";
               }

               file << "]\n";

               file << "    accelerations: [";

               for (std::size_t i = 0;
                    i < point.accelerations.size();
                    ++i)
               {
                    file << point.accelerations[i];

                    if (i + 1 < point.accelerations.size())
                         file << ", ";
               }

               file << "]\n";

               file << "    time_from_start:\n";
               file << "      sec: "
                    << point.time_from_start.sec << "\n";
               file << "      nanosec: "
                    << point.time_from_start.nanosec << "\n";
          }

          file.close();

          return true;
     }

     inline bool loadTrajectory(
         const std::string &name,
         trajectory_msgs::msg::JointTrajectory &trajectory)
     {
          const std::filesystem::path filepath =
              std::filesystem::current_path() / "src" / "meca500_tasks" / "trajectories" / (name + ".yaml");

          if (!std::filesystem::exists(filepath))
               return false;

          try
          {
               const YAML::Node root =
                   YAML::LoadFile(filepath.string());

               trajectory.joint_names.clear();
               trajectory.points.clear();

               // ---------------------------------------------------------
               // JOINT NAMES
               // ---------------------------------------------------------

               for (const auto &joint : root["joint_names"])
               {
                    trajectory.joint_names.push_back(
                        joint.as<std::string>());
               }

               // ---------------------------------------------------------
               // TRAJECTORY POINTS
               // ---------------------------------------------------------

               for (const auto &saved_point : root["points"])
               {
                    trajectory_msgs::msg::JointTrajectoryPoint point;

                    for (const auto &value :
                         saved_point["positions"])
                    {
                         point.positions.push_back(
                             value.as<double>());
                    }

                    for (const auto &value :
                         saved_point["velocities"])
                    {
                         point.velocities.push_back(
                             value.as<double>());
                    }

                    for (const auto &value :
                         saved_point["accelerations"])
                    {
                         point.accelerations.push_back(
                             value.as<double>());
                    }

                    point.time_from_start.sec =
                        saved_point["time_from_start"]["sec"]
                            .as<int32_t>();

                    point.time_from_start.nanosec =
                        saved_point["time_from_start"]["nanosec"]
                            .as<uint32_t>();

                    trajectory.points.push_back(point);
               }

               return !trajectory.points.empty();
          }
          catch (const std::exception &)
          {
               return false;
          }
     }

} // namespace meca500_tasks