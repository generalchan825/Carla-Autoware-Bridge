cmake_minimum_required(VERSION 3.14)
project(autoware_perception_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/TrafficLight.msg"
  # "msg/TrafficLightArray.msg"
  # "msg/TrafficLightElement.msg"
  # "msg/TrafficSignal.msg"
  # "msg/TrafficSignalArray.msg"
  "msg/BoundingBox.idl"
  "msg/BoundingBoxArray.idl"
  "msg/ClassifiedRoi.idl"
  "msg/ClassifiedRoiArray.idl"
  "msg/DetectedObject.idl"
  "msg/DetectedObjectKinematics.idl"
  "msg/DetectedObjects.idl"
  "msg/LookingTrafficSignal.idl"
  "msg/ObjectClassification.idl"
  "msg/PointClusters.idl"
  "msg/PointXYZIF.idl"
  "msg/PredictedObject.idl"
  "msg/PredictedObjectKinematics.idl"
  "msg/PredictedObjects.idl"
  "msg/PredictedPath.idl"
  "msg/Shape.idl"
  "msg/TrackedObject.idl"
  "msg/TrackedObjectKinematics.idl"
  "msg/TrackedObjects.idl"
  "msg/TrafficLight.idl"
  "msg/TrafficLightRoi.idl"
  "msg/TrafficLightRoiArray.idl"
  "msg/TrafficSignal.idl"
  "msg/TrafficSignalArray.idl"
  "msg/TrafficSignalStamped.idl"
  "msg/TrafficSignalWithJudge.idl"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    unique_identifier_msgs
    "autoware_geometry_msgs"
    sensor_msgs
)

ament_auto_package()
