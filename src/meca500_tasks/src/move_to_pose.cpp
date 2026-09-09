#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char* argv[])
{
    // 1. Initialise ROS and create our node.
    rclcpp::init(argc, argv);

    auto const node = std::make_shared<rclcpp::Node>(
        "meca_move_to_pose",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true));

    auto const logger = node->get_logger();

    // Process incoming ROS messages while we wait for MoveIt.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&executor]() { executor.spin(); });

    int exit_code = 1;

    try {
        // 2. Connect to our existing MoveIt planning group.
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, "meca_arm");

        move_group_interface.setPoseReferenceFrame("world");
        move_group_interface.setEndEffectorLink("link_6");

        move_group_interface.setPlanningPipelineId("ompl");
        move_group_interface.setPlannerId("RRTConnectkConfigDefault");
        move_group_interface.setPlanningTime(5.0);

        move_group_interface.setMaxVelocityScalingFactor(0.2);
        move_group_interface.setMaxAccelerationScalingFactor(0.2);

        // Our captured target was rounded, so allow small tolerances.
        move_group_interface.setGoalPositionTolerance(0.001);
        move_group_interface.setGoalOrientationTolerance(0.01);

        // 3. Read the target pose from ROS parameters.
        double x  = node->get_parameter("x").as_double();
        double y  = node->get_parameter("y").as_double();
        double z  = node->get_parameter("z").as_double();

        double qx = node->get_parameter("qx").as_double();
        double qy = node->get_parameter("qy").as_double();
        double qz = node->get_parameter("qz").as_double();
        double qw = node->get_parameter("qw").as_double();

        geometry_msgs::msg::Pose target_pose;

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;

        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;

        if (!move_group_interface.getCurrentState(10.0)) {
            RCLCPP_ERROR(logger, "No current robot state received.");
        } else {
            move_group_interface.setStartStateToCurrentState();
            move_group_interface.setPoseTarget(target_pose);

            // 4. Ask MoveIt for a plan.
            auto const [success, plan] = [&move_group_interface] {
                MoveGroupInterface::Plan result;
                auto const success = static_cast<bool>(
                    move_group_interface.plan(result));

                return std::make_pair(success, result);
            }();

            // 5. Inspect the result. Execution is not enabled yet.
            if (success) {
                RCLCPP_INFO(logger, "Plan found. Executing...");

                const auto result = move_group_interface.execute(plan);

                if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(logger, "Target reached successfully.");
        
                    exit_code = 0;
                } else {
                    RCLCPP_ERROR(logger, "Execution failed.");
                }
            } else {
                RCLCPP_ERROR(logger, "Planning failed. Nothing executed.");
            }
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(logger, "Task failed: %s", error.what());
    }

    // 6. Stop message processing and shut down.
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();

    return exit_code;
}