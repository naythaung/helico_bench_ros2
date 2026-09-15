#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <shape_msgs/msg/mesh.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "helico_environment_loader");

    auto logger = node->get_logger();

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    moveit_msgs::msg::CollisionObject breadboard;

    // IMPORTANT:
    // The breadboard is positioned relative to the same frame
    // that also parents the Meca and station anchor frames.
    breadboard.header.frame_id = "bench_breadboard";
    breadboard.id = "breadboard";

    const std::string mesh_resource =
        "package://meca500_scene_description/"
        "meshes/onshape/"
        "MB3060_M___Aluminium_Breadboard_300_x_600_x_12_7mm_MB3030_M_1.stl";

    shapes::Mesh* mesh =
        shapes::createMeshFromResource(mesh_resource);

    if (!mesh)
    {
        RCLCPP_ERROR(
            logger,
            "Failed to load breadboard mesh.");

        rclcpp::shutdown();
        return 1;
    }

    shape_msgs::msg::Mesh mesh_msg;

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);

    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);

    delete mesh;

    // This is the SAME mesh-local transform that the breadboard
    // previously had in the URDF.
    geometry_msgs::msg::Pose breadboard_pose;

    breadboard_pose.position.x = 0.0;
    breadboard_pose.position.y = -0.0127;
    breadboard_pose.position.z = 0.0;

    breadboard_pose.orientation.x = 0.0;
    breadboard_pose.orientation.y = 0.0;
    breadboard_pose.orientation.z = 0.0;
    breadboard_pose.orientation.w = 1.0;

    breadboard.meshes.push_back(mesh_msg);
    breadboard.mesh_poses.push_back(breadboard_pose);

    breadboard.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    const bool success =
        planning_scene_interface.applyCollisionObject(
            breadboard);

    if (success)
    {
        RCLCPP_INFO(
            logger,
            "Breadboard added to MoveIt world.");
    }
    else
    {
        RCLCPP_ERROR(
            logger,
            "Failed to add breadboard to MoveIt world.");
    }

    rclcpp::shutdown();

    return success ? 0 : 1;
}
