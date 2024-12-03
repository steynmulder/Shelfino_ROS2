cmake_minimum_required(VERSION 3.8)
project(path_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(path_interface
  "msg/Graph.msg"
  "msg/GraphEdge.msg"
  "msg/GraphNode.msg"
  "srv/GenerateGraph.srv"
 )
ament_package()
