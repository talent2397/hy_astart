# ipc library - header-only
add_library(ipc INTERFACE)
target_include_directories(ipc INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(ipc INTERFACE common rt pthread)
