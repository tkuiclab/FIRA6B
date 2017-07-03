
##############################################################################
# Sections
##############################################################################


##############################################################################
# Sources
##############################################################################



##############################################################################
# Binaries
##############################################################################


add_executable(FIRA_strategy strategy_nodeHandle.cpp strategy.cpp FIRA_behavior.cpp FIRA_pathplan.cpp)
target_link_libraries(FIRA_strategy ${catkin_LIBRARIES} BaseNodeLib)
add_dependencies(FIRA_strategy fira_sim_generate_messages_cpp)
