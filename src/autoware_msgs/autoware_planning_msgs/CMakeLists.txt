cmake_minimum_required(VERSION 3.5)
project(autoware_planning_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(autoware_mapping_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LaneletPrimitive.msg"
  "msg/LaneletRoute.msg"
  "msg/LaneletSegment.msg"
  "msg/PoseWithUuidStamped.msg"
  "action/PlannerCostmap.idl"
  "action/PlanTrajectory.idl"
  "action/RecordTrajectory.idl"
  "action/ReplayTrajectory.idl"
  "msg/HADMapRoute.idl"
  "msg/OrderMovement.idl"
  "msg/Route.idl"
  "msg/Trajectory.idl"
  "msg/TrajectoryPoint.idl"
  "msg/Path.idl"
  "msg/PathPoint.idl"
  "msg/PathWithLaneId.idl"
  "msg/PathPointWithLaneId.idl"
  "srv/ModifyTrajectory.idl"
  DEPENDENCIES
    geometry_msgs
    std_msgs
    unique_identifier_msgs
    builtin_interfaces
    action_msgs
    nav_msgs
    autoware_mapping_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
