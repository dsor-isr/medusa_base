cmake_minimum_required(VERSION 3.1)

project(comms_msgs)
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs message_generation)

add_message_files(DIRECTORY msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
  )
