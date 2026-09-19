#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include "meca500_interfaces/srv/list_poses.hpp"

class WorkbenchServer : public rclcpp::Node
{
public:
    WorkbenchServer()
        : Node("meca500_workbench_server")
    {
        list_poses_service_ =
            this->create_service<meca500_interfaces::srv::ListPoses>(
                "/meca/list_poses",
                std::bind(
                    &WorkbenchServer::handleListPoses,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        RCLCPP_INFO(
            this->get_logger(),
            "Meca500 workbench server ready.");
    }

private:
    void handleListPoses(
        const std::shared_ptr<
            meca500_interfaces::srv::ListPoses::Request>,
        std::shared_ptr<
            meca500_interfaces::srv::ListPoses::Response> response)
    {
        const std::string poses_file =
            std::filesystem::current_path().string() +
            "/src/meca500_tasks/config/poses.yaml";

        try
        {
            YAML::Node poses =
                YAML::LoadFile(poses_file);

            response->names.clear();

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

    rclcpp::Service<
        meca500_interfaces::srv::ListPoses>::SharedPtr
        list_poses_service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<WorkbenchServer>());

    rclcpp::shutdown();

    return 0;
}
