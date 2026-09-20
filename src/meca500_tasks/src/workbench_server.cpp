#include <algorithm>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <yaml-cpp/yaml.h>

#include "meca500_interfaces/srv/capture_pose.hpp"
#include "meca500_interfaces/srv/delete_pose.hpp"
#include "meca500_interfaces/srv/list_poses.hpp"

#include "meca500_interfaces/srv/delete_trajectory.hpp"
#include "meca500_interfaces/srv/inspect_trajectory.hpp"
#include "meca500_interfaces/srv/list_trajectories.hpp"

class WorkbenchServer : public rclcpp::Node
{
public:
    // ---------------------------------------------------------
    // CONSTRUCTOR
    // ---------------------------------------------------------

    WorkbenchServer()
        : Node("meca500_workbench_server")
    {
        // -----------------------------------------------------
        // POSE SERVICES
        // -----------------------------------------------------

        list_poses_service_ =
            this->create_service<
                meca500_interfaces::srv::ListPoses>(
                "/meca/list_poses",
                std::bind(
                    &WorkbenchServer::handleListPoses,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        capture_pose_service_ =
            this->create_service<
                meca500_interfaces::srv::CapturePose>(
                "/meca/capture_pose",
                std::bind(
                    &WorkbenchServer::handleCapturePose,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        delete_pose_service_ =
            this->create_service<
                meca500_interfaces::srv::DeletePose>(
                "/meca/delete_pose",
                std::bind(
                    &WorkbenchServer::handleDeletePose,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        // -----------------------------------------------------
        // TRAJECTORY SERVICES
        // -----------------------------------------------------

        list_trajectories_service_ =
            this->create_service<
                meca500_interfaces::srv::ListTrajectories>(
                "/meca/list_trajectories",
                std::bind(
                    &WorkbenchServer::handleListTrajectories,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        inspect_trajectory_service_ =
            this->create_service<
                meca500_interfaces::srv::InspectTrajectory>(
                "/meca/inspect_trajectory",
                std::bind(
                    &WorkbenchServer::handleInspectTrajectory,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        delete_trajectory_service_ =
            this->create_service<
                meca500_interfaces::srv::DeleteTrajectory>(
                "/meca/delete_trajectory",
                std::bind(
                    &WorkbenchServer::handleDeleteTrajectory,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        // -----------------------------------------------------
        // JOINT STATE SUBSCRIPTION
        // -----------------------------------------------------

        joint_state_subscription_ =
            this->create_subscription<
                sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                [this](
                    const sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    latest_joint_state_ = *msg;
                    have_joint_state_ = true;
                });

        RCLCPP_INFO(
            this->get_logger(),
            "Meca500 workbench server ready.");
    }

private:
    // =========================================================
    // PATH HELPERS
    // =========================================================

    std::filesystem::path posesPath() const
    {
        return std::filesystem::current_path() / "src" / "meca500_tasks" / "config" / "poses.yaml";
    }

    std::filesystem::path trajectoriesDirectory() const
    {
        return std::filesystem::current_path() / "src" / "meca500_tasks" / "trajectories";
    }

    // =========================================================
    // LIST POSES
    // =========================================================

    void handleListPoses(
        const std::shared_ptr<
            meca500_interfaces::srv::ListPoses::Request>,
        std::shared_ptr<
            meca500_interfaces::srv::ListPoses::Response>
            response)
    {
        try
        {
            response->names.clear();
            response->types.clear();

            const auto poses_path =
                posesPath();

            if (!std::filesystem::exists(poses_path))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Pose library does not exist yet.");

                return;
            }

            const YAML::Node poses =
                YAML::LoadFile(
                    poses_path.string());

            for (const auto &pose : poses)
            {
                response->names.push_back(
                    pose.first.as<std::string>());

                if (pose.second["type"])
                {
                    response->types.push_back(
                        pose.second["type"].as<std::string>());
                }
                else
                {
                    response->types.push_back(
                        "unknown");
                }
            }

            RCLCPP_INFO(
                this->get_logger(),
                "Returned %zu saved poses.",
                response->names.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to load poses: %s",
                e.what());
        }
    }

    // =========================================================
    // CAPTURE CURRENT POSE
    // =========================================================

    void handleCapturePose(
        const std::shared_ptr<
            meca500_interfaces::srv::CapturePose::Request>
            request,
        std::shared_ptr<
            meca500_interfaces::srv::CapturePose::Response>
            response)
    {
        try
        {
            if (request->name.empty())
            {
                response->success = false;
                response->message =
                    "Pose name cannot be empty.";

                return;
            }

            if (!have_joint_state_)
            {
                response->success = false;
                response->message =
                    "No joint state received yet.";

                return;
            }

            if (latest_joint_state_.name.size() !=
                latest_joint_state_.position.size())
            {
                response->success = false;
                response->message =
                    "Joint state names and positions have different sizes.";

                return;
            }

            const std::vector<std::string> expected_joints = {
                "meca_axis_1",
                "meca_axis_2",
                "meca_axis_3",
                "meca_axis_4",
                "meca_axis_5",
                "meca_axis_6"};

            std::vector<double> joint_values;

            for (const auto &expected_joint : expected_joints)
            {
                bool found = false;

                for (std::size_t i = 0;
                     i < latest_joint_state_.name.size();
                     ++i)
                {
                    if (latest_joint_state_.name[i] ==
                        expected_joint)
                    {
                        joint_values.push_back(
                            latest_joint_state_.position[i]);

                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    response->success = false;
                    response->message =
                        "Missing joint state for " +
                        expected_joint;

                    return;
                }
            }

            if (joint_values.size() != 6)
            {
                response->success = false;
                response->message =
                    "Expected exactly 6 Meca500 joint values.";

                return;
            }

            const auto poses_path =
                posesPath();

            YAML::Node poses;

            if (std::filesystem::exists(poses_path))
            {
                poses =
                    YAML::LoadFile(
                        poses_path.string());
            }

            YAML::Node pose;

            pose["type"] =
                "joint";

            for (double value : joint_values)
            {
                pose["joints"].push_back(
                    value);
            }

            poses[request->name] =
                pose;

            std::ofstream file(
                poses_path);

            if (!file.is_open())
            {
                response->success = false;
                response->message =
                    "Could not open poses.yaml for writing.";

                return;
            }

            file << poses;
            file.close();

            response->success = true;

            response->message =
                "Pose '" +
                request->name +
                "' saved.";

            RCLCPP_INFO(
                this->get_logger(),
                "Saved pose '%s'.",
                request->name.c_str());
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message =
                e.what();

            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to capture pose: %s",
                e.what());
        }
    }

    // =========================================================
    // DELETE POSE
    // =========================================================

    void handleDeletePose(
        const std::shared_ptr<
            meca500_interfaces::srv::DeletePose::Request>
            request,
        std::shared_ptr<
            meca500_interfaces::srv::DeletePose::Response>
            response)
    {
        try
        {
            if (request->name.empty())
            {
                response->success = false;
                response->message =
                    "Pose name cannot be empty.";

                return;
            }

            const auto poses_path =
                posesPath();

            if (!std::filesystem::exists(poses_path))
            {
                response->success = false;
                response->message =
                    "Pose library does not exist.";

                return;
            }

            YAML::Node poses =
                YAML::LoadFile(
                    poses_path.string());

            if (!poses[request->name])
            {
                response->success = false;
                response->message =
                    "Pose '" +
                    request->name +
                    "' does not exist.";

                return;
            }

            poses.remove(
                request->name);

            std::ofstream file(
                poses_path);

            if (!file.is_open())
            {
                response->success = false;
                response->message =
                    "Could not open poses.yaml for writing.";

                return;
            }

            file << poses;
            file.close();

            response->success = true;

            response->message =
                "Pose '" +
                request->name +
                "' deleted.";

            RCLCPP_INFO(
                this->get_logger(),
                "Deleted pose '%s'.",
                request->name.c_str());
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message =
                e.what();

            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to delete pose: %s",
                e.what());
        }
    }

    // =========================================================
    // LIST TRAJECTORIES
    // =========================================================

    void handleListTrajectories(
        const std::shared_ptr<
            meca500_interfaces::srv::ListTrajectories::Request>,
        std::shared_ptr<
            meca500_interfaces::srv::ListTrajectories::Response>
            response)
    {
        try
        {
            response->names.clear();

            const auto directory =
                trajectoriesDirectory();

            if (!std::filesystem::exists(directory))
            {
                return;
            }

            for (const auto &entry :
                 std::filesystem::directory_iterator(directory))
            {
                if (!entry.is_regular_file())
                {
                    continue;
                }

                const auto path =
                    entry.path();

                if (path.extension() == ".yaml")
                {
                    response->names.push_back(
                        path.stem().string());
                }
            }

            std::sort(
                response->names.begin(),
                response->names.end());

            RCLCPP_INFO(
                this->get_logger(),
                "Returned %zu saved trajectories.",
                response->names.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to list trajectories: %s",
                e.what());
        }
    }

    // =========================================================
    // INSPECT TRAJECTORY
    // =========================================================

    void handleInspectTrajectory(
        const std::shared_ptr<
            meca500_interfaces::srv::InspectTrajectory::Request>
            request,
        std::shared_ptr<
            meca500_interfaces::srv::InspectTrajectory::Response>
            response)
    {
        try
        {
            if (request->name.empty())
            {
                response->success = false;
                response->message =
                    "Trajectory name cannot be empty.";

                return;
            }

            const std::filesystem::path filepath =
                trajectoriesDirectory() / (request->name + ".yaml");

            if (!std::filesystem::exists(filepath))
            {
                response->success = false;

                response->message =
                    "Trajectory '" +
                    request->name +
                    "' does not exist.";

                return;
            }

            const YAML::Node root =
                YAML::LoadFile(
                    filepath.string());

            // -------------------------------------------------
            // START / TARGET
            // -------------------------------------------------

            if (root["start_pose"])
            {
                response->start_pose =
                    root["start_pose"]
                        .as<std::string>();
            }

            if (root["target_pose"])
            {
                response->target_pose =
                    root["target_pose"]
                        .as<std::string>();
            }

            // -------------------------------------------------
            // PLANNING SETTINGS
            // -------------------------------------------------

            if (root["planning"])
            {
                const YAML::Node planning =
                    root["planning"];

                if (planning["num_candidates"])
                {
                    response->num_candidates =
                        planning["num_candidates"]
                            .as<int>();
                }

                if (planning["minimum_required_clearance"])
                {
                    response->minimum_required_clearance =
                        planning["minimum_required_clearance"]
                            .as<double>();
                }

                if (planning["weight_clearance"])
                {
                    response->weight_clearance =
                        planning["weight_clearance"]
                            .as<double>();
                }

                if (planning["weight_smoothness"])
                {
                    response->weight_smoothness =
                        planning["weight_smoothness"]
                            .as<double>();
                }

                if (planning["weight_path_length"])
                {
                    response->weight_path_length =
                        planning["weight_path_length"]
                            .as<double>();
                }

                if (planning["weight_duration"])
                {
                    response->weight_duration =
                        planning["weight_duration"]
                            .as<double>();
                }
            }

            // -------------------------------------------------
            // TRAJECTORY METRICS
            // -------------------------------------------------

            if (root["metrics"])
            {
                const YAML::Node metrics =
                    root["metrics"];

                if (metrics["minimum_clearance"])
                {
                    response->minimum_clearance =
                        metrics["minimum_clearance"]
                            .as<double>();
                }

                if (metrics["closest_object_a"])
                {
                    response->closest_object_a =
                        metrics["closest_object_a"]
                            .as<std::string>();
                }

                if (metrics["closest_object_b"])
                {
                    response->closest_object_b =
                        metrics["closest_object_b"]
                            .as<std::string>();
                }

                if (metrics["smoothness"])
                {
                    response->smoothness =
                        metrics["smoothness"]
                            .as<double>();
                }

                if (metrics["path_length"])
                {
                    response->path_length =
                        metrics["path_length"]
                            .as<double>();
                }

                if (metrics["duration"])
                {
                    response->duration =
                        metrics["duration"]
                            .as<double>();
                }
            }

            response->success =
                true;

            response->message =
                "Trajectory '" +
                request->name +
                "' loaded.";

            RCLCPP_INFO(
                this->get_logger(),
                "Inspected trajectory '%s'.",
                request->name.c_str());
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message =
                e.what();

            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to inspect trajectory: %s",
                e.what());
        }
    }

    // =========================================================
    // DELETE TRAJECTORY
    // =========================================================

    void handleDeleteTrajectory(
        const std::shared_ptr<
            meca500_interfaces::srv::DeleteTrajectory::Request>
            request,
        std::shared_ptr<
            meca500_interfaces::srv::DeleteTrajectory::Response>
            response)
    {
        try
        {
            if (request->name.empty())
            {
                response->success = false;
                response->message =
                    "Trajectory name cannot be empty.";

                return;
            }

            const std::filesystem::path filepath =
                trajectoriesDirectory() / (request->name + ".yaml");

            if (!std::filesystem::exists(filepath))
            {
                response->success = false;

                response->message =
                    "Trajectory '" +
                    request->name +
                    "' does not exist.";

                return;
            }

            if (!std::filesystem::remove(filepath))
            {
                response->success = false;
                response->message =
                    "Failed to delete trajectory file.";

                return;
            }

            response->success =
                true;

            response->message =
                "Trajectory '" +
                request->name +
                "' deleted.";

            RCLCPP_INFO(
                this->get_logger(),
                "Deleted trajectory '%s'.",
                request->name.c_str());
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message =
                e.what();

            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to delete trajectory: %s",
                e.what());
        }
    }

    // =========================================================
    // DATA STORED BY THIS NODE
    // =========================================================

    sensor_msgs::msg::JointState
        latest_joint_state_;

    bool have_joint_state_ =
        false;

    // =========================================================
    // ROS INTERFACES
    // =========================================================

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscription_;

    // Pose services

    rclcpp::Service<
        meca500_interfaces::srv::ListPoses>::SharedPtr
        list_poses_service_;

    rclcpp::Service<
        meca500_interfaces::srv::CapturePose>::SharedPtr
        capture_pose_service_;

    rclcpp::Service<
        meca500_interfaces::srv::DeletePose>::SharedPtr
        delete_pose_service_;

    // Trajectory services

    rclcpp::Service<
        meca500_interfaces::srv::ListTrajectories>::SharedPtr
        list_trajectories_service_;

    rclcpp::Service<
        meca500_interfaces::srv::InspectTrajectory>::SharedPtr
        inspect_trajectory_service_;

    rclcpp::Service<
        meca500_interfaces::srv::DeleteTrajectory>::SharedPtr
        delete_trajectory_service_;
};

// =============================================================
// MAIN
// =============================================================

int main(int argc, char **argv)
{
    rclcpp::init(
        argc,
        argv);

    auto server =
        std::make_shared<WorkbenchServer>();

    rclcpp::spin(
        server);

    rclcpp::shutdown();

    return 0;
}