cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME graph_traversal)
project(${PROJECT_NAME})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-fdiagnostics-color=always)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenMP REQUIRED)

add_library(utils STATIC src/utils.cpp)
add_executable(bfs src/bfs.cpp)
add_executable(occupancy src/occupancy_grid.cpp)
add_executable(move src/move.cpp)
add_executable(octomap_pub src/octomap_publisher.cpp)

ament_target_dependencies(bfs  rclcpp octomap octomap_msgs nav2_msgs std_msgs)
ament_target_dependencies(occupancy  rclcpp octomap octomap_msgs nav2_msgs)
ament_target_dependencies(move  rclcpp octomap octomap_msgs nav2_msgs std_msgs std_srvs crazyflie_interfaces sensor_msgs)
ament_target_dependencies(octomap_pub  rclcpp octomap octomap_msgs nav2_msgs std_srvs crazyflie_interfaces)


target_link_libraries(utils
  yaml-cpp
)


add_executable(test_utils src/test_utils.cpp)
target_link_libraries(test_utils
  utils
)
target_include_directories(test_utils PRIVATE include)
add_test(NAME TestUtils
        COMMAND test_utils)


target_include_directories(bfs PRIVATE include)
target_link_libraries(bfs
  ${OpenCV_LIBRARIES}
  octomap
  utils
)

target_include_directories(occupancy PRIVATE include)
target_link_libraries(occupancy
  ${OpenCV_LIBRARIES}
  octomap
)


target_include_directories(move PRIVATE include)
target_link_libraries(move
  ${OpenCV_LIBRARIES}
  octomap
  OpenMP::OpenMP_CXX
  utils
)

target_include_directories(octomap_pub PRIVATE include)
target_link_libraries(octomap_pub
  ${OpenCV_LIBRARIES}
  octomap
)

install(TARGETS
  utils
  bfs
  occupancy
  move
  octomap_pub
  DESTINATION lib/${PROJECT_NAME})

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
