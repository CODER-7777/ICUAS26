cmake_minimum_required(VERSION 3.8)
project(octomap_map_generator)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #Important if you want code completion IDEs and neovim
add_compile_options(-fdiagnostics-color=always)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# ...
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

add_executable(waypoint_extractor_node src/waypoint_extractor_node.cpp src/graph_solver.cpp)
ament_target_dependencies(waypoint_extractor_node
  rclcpp
  octomap_msgs
  octomap
)

# 2. Link your external (non-ament) libraries like OpenCV
target_link_libraries(waypoint_extractor_node
  octomap
  ${OpenCV_LIBRARIES}
  # You can also link the specific components if ${OpenCV_LIBRARIES} doesn't work
  # OpenCV::opencv_core
  # OpenCV::opencv_imgproc
  # OpenCV::opencv_imgcodecs
)
# ...

install(TARGETS
  waypoint_extractor_node
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
