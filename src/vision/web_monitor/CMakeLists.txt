add_executable(monitor interface_node.cpp monitor.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
