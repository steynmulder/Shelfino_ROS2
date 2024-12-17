cmake_minimum_required(VERSION 3.8)
project(path_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(path_interface
  "msg/Graph.msg"
  "msg/GraphEdge.msg"
  "msg/GraphNode.msg"
  "msg/PathArray.msg"
  "srv/GenerateGraph.srv"
  "srv/MoveRobots.srv"
  DEPENDENCIES nav_msgs
 )

ament_export_dependencies(nav_msgs)
ament_package()
