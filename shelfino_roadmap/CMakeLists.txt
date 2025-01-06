cmake_minimum_required(VERSION 3.8)
project(shelfino_roadmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(path_interface REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(include/shelfino_roadmap)

add_executable(send_roadmap src/send_roadmap.cpp src/GEdge.cpp src/GVertex.cpp src/Dubins.cpp)
ament_target_dependencies(send_roadmap rclcpp obstacles_msgs geometry_msgs path_interface)

add_executable(astar_planner src/astar_planner.cpp src/GEdge.cpp src/GVertex.cpp src/Astar.cpp src/Dubins.cpp)
ament_target_dependencies(astar_planner rclcpp obstacles_msgs geometry_msgs path_interface nav_msgs)

add_executable(rrtstar_planner src/rrtstar_planner.cpp src/RRTstar.cpp src/GVertex.cpp src/Dubins.cpp)
ament_target_dependencies(rrtstar_planner RRTstar rclcpp obstacles_msgs geometry_msgs path_interface nav_msgs)

# target_include_directories(astar PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

install(DIRECTORY include/
  DESTINATION include/
)

INSTALL(TARGETS
  send_roadmap
  astar_planner
  rrtstar_planner
  
  DESTINATION lib/${PROJECT_NAME})


ament_package()
