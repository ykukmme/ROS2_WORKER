cmake_minimum_required(VERSION 3.8)
project(ai_worker_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/PlacementZone.msg"
  "msg/SortRule.msg"
  "srv/ScanZones.srv"
  "srv/PrepareAutoSort.srv"
  "srv/StartTask.srv"
  "srv/SessionStatus.srv"
  "srv/SortOnce.srv"
  "srv/TaskStatus.srv"
  "srv/CancelTask.srv"
  "srv/DetectObjects.srv"
  DEPENDENCIES geometry_msgs vision_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
