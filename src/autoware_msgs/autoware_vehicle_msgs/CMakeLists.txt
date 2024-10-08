# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_vehicle_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_mapping_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlModeCommand.idl"
  "msg/ControlModeReport.idl"
  "msg/Engage.idl"
  "msg/GearCommand.idl"
  "msg/GearReport.idl"
  "msg/HandBrakeCommand.idl"
  "msg/HandBrakeReport.idl"
  "msg/HazardLightsCommand.idl"
  "msg/HazardLightsReport.idl"
  "msg/HeadlightsCommand.idl"
  "msg/HeadlightsReport.idl"
  "msg/HornCommand.idl"
  "msg/HornReport.idl"
  "msg/RawControlCommand.idl"
  "msg/StationaryLockingCommand.idl"
  "msg/SteeringReport.idl"
  "msg/TurnIndicatorsCommand.idl"
  "msg/TurnIndicatorsReport.idl"
  "msg/VehicleControlCommand.idl"
  "msg/VehicleKinematicState.idl"
  "msg/VehicleOdometry.idl"
  "msg/VehicleStateCommand.idl"
  "msg/VehicleStateReport.idl"
  "msg/VelocityReport.idl"
  "msg/WheelEncoder.idl"
  "msg/WipersCommand.idl"
  "msg/WipersReport.idl"
  "srv/AutonomyModeChange.idl"
  "srv/ControlModeCommand.srv"
  DEPENDENCIES
    "autoware_planning_msgs"
    "autoware_mapping_msgs"
    "builtin_interfaces"
    "std_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
