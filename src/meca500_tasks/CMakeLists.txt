cmake_minimum_required(VERSION 3.20)
project(meca500_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Move-to-pose executable plus trajectory evaluation functions
add_executable(move_to_pose
  src/move_to_pose.cpp
  src/trajectory_evaluator.cpp
)

target_compile_features(move_to_pose PUBLIC cxx_std_17)

# Allows:
# #include "meca500_tasks/trajectory_evaluator.hpp"
target_include_directories(move_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(move_to_pose PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
)

install(
  TARGETS move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()