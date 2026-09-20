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
#include "meca500_interfaces/srv/list_poses.hpp"


class WorkbenchServer : public rclcpp::Node
{
public:
    // ---------------------------------------------------------
    // CONSTRUCTOR
    // Runs once when the WorkbenchServer node is created
    // ---------------------------------------------------------

    WorkbenchServer()
        : Node("meca500_workbench_server")
    {
        // Service: list all saved poses
        list_poses_service_ =
            this->create_service<meca500_interfaces::srv::ListPoses>(
                "/meca/list_poses",
                std::bind(
                    &WorkbenchServer::handleListPoses,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        // Subscribe to the robot's live joint states
        joint_state_subscription_ =
            this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                [this](
                    const sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    latest_joint_state_ = *msg;
                    have_joint_state_ = true;
                });

        // Service: capture the robot's current pose
        capture_pose_service_ =
            this->create_service<meca500_interfaces::srv::CapturePose>(
                "/meca/capture_pose",
                std::bind(
                    &WorkbenchServer::handleCapturePose,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        RCLCPP_INFO(
            this->get_logger(),
            "Meca500 workbench server ready.");
    }


private:
    // ---------------------------------------------------------
    // LIST POSES
    // ---------------------------------------------------------

    void handleListPoses(
        const std::shared_ptr<
            meca500_interfaces::srv::ListPoses::Request>,
        std::shared_ptr<
            meca500_interfaces::srv::ListPoses::Response> response)
    {
        const std::filesystem::path poses_path =
            std::filesystem::current_path()
            / "src"
            / "meca500_tasks"
            / "config"
            / "poses.yaml";

        try
        {
            response->names.clear();

            if (!std::filesystem::exists(poses_path))
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Pose library does not exist yet.");

                return;
            }

            const YAML::Node poses =
                YAML::LoadFile(poses_path.string());

            for (const auto &pose : poses)
            {
                response->names.push_back(
                    pose.first.as<std::string>());
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


    // ---------------------------------------------------------
    // CAPTURE CURRENT POSE
    // ---------------------------------------------------------

    void handleCapturePose(
        const std::shared_ptr<
            meca500_interfaces::srv::CapturePose::Request> request,
        std::shared_ptr<
            meca500_interfaces::srv::CapturePose::Response> response)
    {
        try
        {
            // Make sure a name was supplied
            if (request->name.empty())
            {
                response->success = false;
                response->message =
                    "Pose name cannot be empty.";

                return;
            }

            // Make sure we have received /joint_states
            if (!have_joint_state_)
            {
                response->success = false;
                response->message =
                    "No joint state received yet.";

                return;
            }

            // Safety check before indexing position[]
            if (latest_joint_state_.name.size() !=
                latest_joint_state_.position.size())
            {
                response->success = false;
                response->message =
                    "Joint state names and positions have different sizes.";

                return;
            }

            // The six joints we care about, in the correct order
            const std::vector<std::string> expected_joints = {
                "meca_axis_1",
                "meca_axis_2",
                "meca_axis_3",
                "meca_axis_4",
                "meca_axis_5",
                "meca_axis_6"
            };

            std::vector<double> joint_values;

            // Find each Meca joint in /joint_states
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

            // Location of our pose library
            const std::filesystem::path poses_path =
                std::filesystem::current_path()
                / "src"
                / "meca500_tasks"
                / "config"
                / "poses.yaml";

            YAML::Node poses;

            // Load existing poses if the file already exists
            if (std::filesystem::exists(poses_path))
            {
                poses =
                    YAML::LoadFile(
                        poses_path.string());
            }

            // Create the new pose
            YAML::Node pose;

            pose["type"] = "joint";

            for (double value : joint_values)
            {
                pose["joints"].push_back(value);
            }

            // Add/replace this named pose
            poses[request->name] = pose;

            // Write the whole pose library back to disk
            std::ofstream file(poses_path);

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
            response->message = e.what();

            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to capture pose: %s",
                e.what());
        }
    }


    // ---------------------------------------------------------
    // DATA STORED BY THIS NODE
    // ---------------------------------------------------------

    sensor_msgs::msg::JointState latest_joint_state_;

    bool have_joint_state_ = false;


    // ---------------------------------------------------------
    // ROS INTERFACES
    // ---------------------------------------------------------

    rclcpp::Subscription<
        sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscription_;

    rclcpp::Service<
        meca500_interfaces::srv::ListPoses>::SharedPtr
        list_poses_service_;

    rclcpp::Service<
        meca500_interfaces::srv::CapturePose>::SharedPtr
        capture_pose_service_;
};


// ---------------------------------------------------------
// MAIN
// ---------------------------------------------------------

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto server =
        std::make_shared<WorkbenchServer>();

    rclcpp::spin(server);

    rclcpp::shutdown();

    return 0;
}