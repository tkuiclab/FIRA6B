cmake_minimum_required(VERSION 2.8.3)
project(strategy)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

# add_message_files(
#   FILES  # e.g. Floats.msg HeaderString.msg
# )

# Uncomment if the package has a setup.py
catkin_python_setup()


## Generate added messages and services with any dependencies
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

catkin_install_python(PROGRAMS bin/wheel_strategy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})