cmake_minimum_required(VERSION 2.8.3)
project(sb_jaus)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)

# Catkin Initalization
catkin_python_setup()
catkin_package()

# Declare executables (python ROS nodes)
catkin_install_python(PROGRAMS
    scripts/jaus_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
