# gazebo_bridge - only built when ROS is available
add_executable(run_gazebo_bridge
    app/run_gazebo_bridge.cpp
)

target_include_directories(run_gazebo_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(run_gazebo_bridge
    ipc
    common
    ${GLOG_LIBRARIES}
    ${catkin_LIBRARIES}
    rt
)
