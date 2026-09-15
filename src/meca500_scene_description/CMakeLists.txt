cmake_minimum_required(VERSION 3.20)
project(meca500_scene_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

add_executable(environment_loader
  src/environment_loader.cpp
)

target_compile_features(environment_loader PUBLIC cxx_std_17)

target_link_libraries(environment_loader PRIVATE
  rclcpp::rclcpp
  ${moveit_ros_planning_interface_TARGETS}
  ${moveit_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${shape_msgs_TARGETS}
  ${geometric_shapes_TARGETS}
)

install(
  TARGETS environment_loader
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
