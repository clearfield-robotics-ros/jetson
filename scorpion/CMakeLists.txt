cmake_minimum_required(VERSION 2.8.3)
project(scorpion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)

# add_service_files(
#   FILES
# )

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
