cmake_minimum_required(VERSION 3.8)
project(swarm_viz)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(graph_traversal REQUIRED)

include_directories(include)

add_executable(viz_node src/viz_node.cpp)
ament_target_dependencies(viz_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  graph_traversal
)

# Bresenham LOS + marker construction are tight loops; match the planner's
# release flags so viz_node doesn't drag CPU at default -O2.
target_compile_options(viz_node PRIVATE
  $<$<CONFIG:Release>:-O3 -DNDEBUG -march=native>
  $<$<CONFIG:RelWithDebInfo>:-O3 -DNDEBUG -march=native>
)

install(TARGETS viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
