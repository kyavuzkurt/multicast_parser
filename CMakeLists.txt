cmake_minimum_required(VERSION 3.0.2)
project(multicast_parser)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_runtime
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs
)

# Specify additional locations of header files
# if you have any, relative to this CMakeLists.txt
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )

## Install Python scripts
catkin_install_python(PROGRAMS
  src/multicast_parser/multicast_listener.py
  src/multicast_parser/data_parser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## (Optional) Install launch files if you have any
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## (Optional) Install configuration files if needed
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
