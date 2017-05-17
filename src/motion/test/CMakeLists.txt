cmake_minimum_required(VERSION 2.8.3)
project(test)


add_executable(simulate_strategy_node simulate_strategy.cpp)
target_link_libraries(simulate_strategy_node ${catkin_LIBRARIES})

