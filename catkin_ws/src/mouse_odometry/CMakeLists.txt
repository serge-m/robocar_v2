cmake_minimum_required(VERSION 2.8.3)
project(mouse_odometry)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  rospy
)


################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
)

#############
## Install ##
#############
install(DIRECTORY
  src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

