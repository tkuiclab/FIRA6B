add_executable(interface_node interface_node.cpp interface.cpp)
target_link_libraries(interface_node ${catkin_LIBRARIES})
add_dependencies(interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
