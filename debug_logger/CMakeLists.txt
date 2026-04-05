cmake_minimum_required(VERSION 3.8)
project(debug_logger)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-fdiagnostics-color=always)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(icuas25_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(debug_logger_node src/debug_logger_node.cpp)

ament_target_dependencies(debug_logger_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  crazyflie_interfaces
  icuas25_msgs
  nav_msgs
  octomap_msgs
)

target_link_libraries(debug_logger_node
  yaml-cpp
  ${OCTOMAP_LIBRARIES}
)

install(TARGETS
  debug_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
