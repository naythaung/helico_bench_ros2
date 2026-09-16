#pragma once

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "meca500_tasks/trajectory_evaluator.hpp"

namespace meca500_tasks
{

     inline bool saveTrajectory(
         const std::string &name,
         const trajectory_msgs::msg::JointTrajectory &trajectory,
         const TrajectoryMetrics &metrics)
     {
          // Portable location for runtime-generated files:
          //
          // ~/.ros/helico_bench/trajectories/
          const char *home = std::getenv("HOME");

          if (home == nullptr)
               return false;

          const std::filesystem::path directory =
              std::filesystem::current_path() 
              / "src" 
              / "meca500_tasks" 
              / "trajectories";

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

          for (const auto &joint_name :
               trajectory.joint_names)
          {
               file << "  - " << joint_name << "\n";
          }

          // ---------------------------------------------------------
          // TRAJECTORY POINTS
          // ---------------------------------------------------------

          file << "points:\n";

          for (const auto &point :
               trajectory.points)
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
                    << point.time_from_start.sec
                    << "\n";

               file << "      nanosec: "
                    << point.time_from_start.nanosec
                    << "\n";
          }

          file.close();

          return true;
     }

} // namespace meca500_tasks
