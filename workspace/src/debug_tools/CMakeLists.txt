cmake_minimum_required(VERSION 3.22)
project(debug_tools)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-fdiagnostics-color=always)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(slider_gui_node src/slider_gui_node.cpp)
add_executable(role_publisher src/role_publisher.cpp)
add_executable(connectivity_checker src/connectivity_checker.cpp)

ament_target_dependencies(slider_gui_node
  rclcpp
  sensor_msgs
)

target_link_libraries(slider_gui_node
  Qt5::Widgets
)

ament_target_dependencies(role_publisher
  rclcpp
  sensor_msgs
)

target_link_libraries(role_publisher
  Qt5::Widgets
)

ament_target_dependencies(connectivity_checker
  rclcpp
  geometry_msgs
  octomap
  octomap_msgs
  nav_msgs
)

target_link_libraries(connectivity_checker
  octomap
)

install(TARGETS slider_gui_node role_publisher octomap_binary_publisher grid_visualizer connectivity_checker
  DESTINATION lib/${PROJECT_NAME}
) 


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
