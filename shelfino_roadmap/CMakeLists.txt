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

add_executable(send_roadmap src/send_roadmap.cpp)
ament_target_dependencies(send_roadmap rclcpp obstacles_msgs geometry_msgs)

INSTALL(TARGETS
  send_roadmap
  DESTINATION lib/${PROJECT_NAME})


ament_package()
