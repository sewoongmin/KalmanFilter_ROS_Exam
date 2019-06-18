cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter_exam)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  mavros
  mavros_msgs
  mavros_extras
  message_generation
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#    DEPENDENCIES
#    mavros_msgs
#    std_msgs
#    rospy
# )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES kalman_filter_exam
  # CATKIN_DEPENDS mavros mavros_extras rospy std_msgs
 ## DEPENDS system_lib
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)