# planner library
add_library(planner_lib STATIC
    src/hybrid_a_star.cpp
    src/rs_path.cpp
    src/hybrid_a_star_planner.cpp
)

target_include_directories(planner_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(planner_lib PUBLIC
    common
    ipc
    ${GLOG_LIBRARIES}
    ${OpenMP_CXX_LIBRARIES}
)

# planner executable
add_executable(run_planner
    app/run_planner.cpp
)
target_link_libraries(run_planner
    planner_lib
    ${GLOG_LIBRARIES}
)
