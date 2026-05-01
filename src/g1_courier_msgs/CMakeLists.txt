cmake_minimum_required(VERSION 3.8)
project(g1_courier_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/NavigateToPose.action"
  "action/DockToTable.action"
  "action/PickBox.action"
  "action/PlaceBox.action"
  "action/Retreat.action"
)

set(srv_files
  "srv/SetCarryMode.srv"
  "srv/SetFreeze.srv"
)

set(msg_files
  "msg/MissionStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
