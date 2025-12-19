cmake_minimum_required(VERSION 3.10)
project(OctoMap_CV_Boxes)

set(CMAKE_CXX_STANDARD 14)

# Find OctoMap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

# Find OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
include_directories(${OpenCV_INCLUDE_DIRS})

# Add your executable
add_executable(bound_box bound_box.cpp)

# Link the libraries
target_link_libraries(bound_box
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBS}
)