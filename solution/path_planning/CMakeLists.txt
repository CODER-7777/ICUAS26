cmake_minimum_required(VERSION 3.14)
project(pathplanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add your source files
add_executable(main
    main.cpp
    graph_solver.cpp
)

