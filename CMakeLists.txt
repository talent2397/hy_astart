cmake_minimum_required(VERSION 3.16)
project(hybrid_a_star_system VERSION 1.0.0 LANGUAGES CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# ---- Dependencies ----

# Eigen3
find_package(Eigen3 REQUIRED)

# glog
find_package(glog REQUIRED)

# yaml-cpp
find_package(yaml-cpp QUIET)

# OpenMP (optional)
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    message(STATUS "OpenMP found and enabled")
endif()

# ROS (仅 gazebo_bridge 需要)
find_package(catkin QUIET COMPONENTS roscpp nav_msgs geometry_msgs tf)

# ---- Subdirectories ----
add_subdirectory(src/common)
add_subdirectory(src/ipc)
add_subdirectory(src/planner)
add_subdirectory(src/tracker)

if(catkin_FOUND)
    add_subdirectory(src/gazebo_bridge)
    message(STATUS "ROS found, building gazebo_bridge")
else()
    message(WARNING "ROS not found, skipping gazebo_bridge. Run 'source /opt/ros/<distro>/setup.bash' first")
endif()

# ---- 打印构建摘要 ----
message(STATUS "")
message(STATUS "=== Build Configuration ===")
message(STATUS "  C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "  Eigen3:            ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  glog:              found")
message(STATUS "  OpenMP:            ${OpenMP_CXX_FOUND}")
message(STATUS "  ROS (catkin):      ${catkin_FOUND}")
message(STATUS "")
