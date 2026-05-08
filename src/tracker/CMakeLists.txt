# tracker library
add_library(tracker_lib STATIC
    src/pure_pursuit_tracker.cpp
)

target_include_directories(tracker_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(tracker_lib PUBLIC
    common
    ipc
    ${GLOG_LIBRARIES}
)

# tracker executable
add_executable(run_tracker
    app/run_tracker.cpp
)
target_link_libraries(run_tracker
    tracker_lib
    ${GLOG_LIBRARIES}
)
