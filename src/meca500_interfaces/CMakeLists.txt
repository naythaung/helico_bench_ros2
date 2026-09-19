cmake_minimum_required(VERSION 3.8)
project(meca500_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CapturePose.srv"
  "srv/ListPoses.srv"
  "srv/DeletePose.srv"
  "srv/ListTrajectories.srv"

  "action/PlanTrajectory.action"
  "action/GoToTrajectoryStart.action"
  "action/ExecuteTrajectory.action"
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()