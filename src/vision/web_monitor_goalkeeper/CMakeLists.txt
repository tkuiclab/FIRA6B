add_executable(monitor_goalkeeper interface_node.cpp monitor_goalkeeper.cpp)
target_link_libraries(monitor_goalkeeper ${catkin_LIBRARIES})
add_dependencies(monitor_goalkeeper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
