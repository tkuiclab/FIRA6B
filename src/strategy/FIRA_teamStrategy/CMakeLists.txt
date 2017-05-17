
##############################################################################
# Sections
##############################################################################


##############################################################################
# Sources
##############################################################################



##############################################################################
# Binaries
##############################################################################

add_executable(teamStrategy FIRA_teamStrategy.cpp teamStrategy_nodeHandle.cpp main.cpp)
add_dependencies(teamStrategy FIRA_strategy_gencpp)
target_link_libraries(teamStrategy ${catkin_LIBRARIES} BaseNodeLib)
#add_dependencies(FIRA_strategy FIRA_strategy_generate_messages_cpp)
#add_dependencies(FIRA_strategy FIRA_Sim_generate_messages_cpp)
#add_dependencies(FIRA_teamStrategy fira_sim_generate_messages_cpp)
