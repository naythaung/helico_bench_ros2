#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <algorithm>
#include <chrono>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

moveit_msgs::msg::CollisionObject makeMeshObject(
    const std::string &id,
    const std::string &frame_id,
    const std::string &mesh_resource,
    double x,
    double y,
    double z)
{
    moveit_msgs::msg::CollisionObject object;

    object.header.frame_id = frame_id;
    object.id = id;

    shapes::Mesh *mesh =
        shapes::createMeshFromResource(mesh_resource);

    if (!mesh)
    {
        throw std::runtime_error(
            "Failed to load mesh: " + mesh_resource);
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);

    shape_msgs::msg::Mesh mesh_msg =
        boost::get<shape_msgs::msg::Mesh>(shape_msg);

    delete mesh;

    geometry_msgs::msg::Pose pose;

    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    object.meshes.push_back(mesh_msg);
    object.mesh_poses.push_back(pose);

    object.operation =
        moveit_msgs::msg::CollisionObject::ADD;

    return object;
}

void setAllowedCollision(
    moveit_msgs::msg::AllowedCollisionMatrix &acm,
    const std::string &name_a,
    const std::string &name_b,
    bool allowed)
{
    auto ensure_entry =
        [&acm](const std::string &name)
    {
        auto it = std::find(
            acm.entry_names.begin(),
            acm.entry_names.end(),
            name);

        if (it != acm.entry_names.end())
        {
            return static_cast<std::size_t>(
                std::distance(acm.entry_names.begin(), it));
        }

        const std::size_t old_size =
            acm.entry_names.size();

        acm.entry_names.push_back(name);

        // Add one new column to all existing rows.
        for (auto &row : acm.entry_values)
        {
            row.enabled.push_back(false);
        }

        // Add the new row.
        moveit_msgs::msg::AllowedCollisionEntry new_row;
        new_row.enabled.resize(old_size + 1, false);

        acm.entry_values.push_back(new_row);

        return old_size;
    };

    const std::size_t index_a = ensure_entry(name_a);
    const std::size_t index_b = ensure_entry(name_b);

    acm.entry_values[index_a].enabled[index_b] = allowed;
    acm.entry_values[index_b].enabled[index_a] = allowed;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<rclcpp::Node>(
            "helico_environment_loader");

    auto logger = node->get_logger();

    moveit::planning_interface::PlanningSceneInterface
        planning_scene_interface;

    const std::string mesh_root =
        "package://meca500_scene_description/meshes/onshape/";

    std::vector<moveit_msgs::msg::CollisionObject> objects;

    try
    {
        // =====================================================
        // Breadboard
        // =====================================================

        objects.push_back(
            makeMeshObject(
                "breadboard",
                "bench_breadboard",
                mesh_root +
                    "MB3060_M___Aluminium_Breadboard_300_x_600_x_12_7mm_MB3030_M_1.stl",
                0.0,
                -0.0127,
                0.0));

        // =====================================================
        // 3.5 mm station
        // =====================================================

        objects.push_back(
            makeMeshObject(
                "tip_station_3p5mm_base",
                "tip_station_3p5mm_base_visual",
                mesh_root + "Base.stl",
                -0.0125,
                0.0125,
                0.025));

        objects.push_back(
            makeMeshObject(
                "tip_station_3p5mm_coupling",
                "tip_station_3p5mm_coupling_visual",
                mesh_root + "Coupling_base.stl",
                -0.006,
                0.0,
                0.0));

        objects.push_back(
            makeMeshObject(
                "tip_station_3p5mm_holder",
                "tip_station_3p5mm_holder_visual",
                mesh_root + "_3D_printed.stl",
                0.0,
                -0.0123,
                0.0));

        // =====================================================
        // 4 mm station
        // =====================================================

        objects.push_back(
            makeMeshObject(
                "tip_station_4mm_base",
                "tip_station_4mm_base_visual",
                mesh_root + "Base.stl",
                0.0125,
                0.0125,
                0.025));

        objects.push_back(
            makeMeshObject(
                "tip_station_4mm_coupling",
                "tip_station_4mm_coupling_visual",
                mesh_root + "Coupling_base.stl",
                -0.006,
                0.0,
                0.0));

        objects.push_back(
            makeMeshObject(
                "tip_station_4mm_holder",
                "tip_station_4mm_holder_visual",
                mesh_root + "_3D_printed.stl",
                0.0,
                -0.0123,
                0.0));

        // =====================================================
        // 5 mm station
        // =====================================================

        objects.push_back(
            makeMeshObject(
                "tip_station_5mm_base",
                "tip_station_5mm_base_visual",
                mesh_root + "Base.stl",
                -0.0125,
                0.0125,
                0.025));

        objects.push_back(
            makeMeshObject(
                "tip_station_5mm_coupling",
                "tip_station_5mm_coupling_visual",
                mesh_root + "Coupling_base.stl",
                -0.006,
                0.0,
                0.0));

        objects.push_back(
            makeMeshObject(
                "tip_station_5mm_holder",
                "tip_station_5mm_holder_visual",
                mesh_root + "_3D_printed.stl",
                0.0,
                -0.0123,
                0.0));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            logger,
            "%s",
            e.what());

        rclcpp::shutdown();
        return 1;
    }

    const bool success =
        planning_scene_interface.applyCollisionObjects(
            objects);

    if (!success)
    {
        RCLCPP_ERROR(
            logger,
            "Failed to load Helico environment collision objects.");

        rclcpp::shutdown();
        return 1;
    }

    // =====================================================
    // Preserve existing ACM and add world exception
    // =====================================================

    auto get_scene_client =
        node->create_client<moveit_msgs::srv::GetPlanningScene>(
            "/get_planning_scene");

    if (!get_scene_client->wait_for_service(
            std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(
            logger,
            "/get_planning_scene service not available.");

        rclcpp::shutdown();
        return 1;
    }

    auto request =
        std::make_shared<
            moveit_msgs::srv::GetPlanningScene::Request>();

    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::
            ALLOWED_COLLISION_MATRIX;

    auto future =
        get_scene_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            node,
            future,
            std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(
            logger,
            "Failed to retrieve current PlanningScene ACM.");

        rclcpp::shutdown();
        return 1;
    }

    auto response = future.get();

    auto acm =
        response->scene.allowed_collision_matrix;

    // Preserve everything already in MoveIt and add ONLY this exception.
    setAllowedCollision(
        acm,
        "base_link",
        "breadboard",
        true);

    moveit_msgs::msg::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;
    planning_scene_msg.allowed_collision_matrix = acm;

    const bool acm_success =
        planning_scene_interface.applyPlanningScene(
            planning_scene_msg);

    if (!acm_success)
    {
        RCLCPP_ERROR(
            logger,
            "Failed to update Allowed Collision Matrix.");

        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(
        logger,
        "Loaded %zu Helico environment collision objects.",
        objects.size());

    RCLCPP_INFO(
        logger,
        "Allowed intentional collision: base_link <-> breadboard.");

    rclcpp::shutdown();

    return 0;
}