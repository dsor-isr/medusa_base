cmake_minimum_required(VERSION 3.1)
project(data_serializer)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 medusa_msgs
 rospy
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS
)
