cmake_minimum_required(VERSION 3.1)
project(status_aggregator)

find_package(catkin REQUIRED
 COMPONENTS
 std_msgs
 std_srvs 
 medusa_msgs
 roscpp
 diagnostic_msgs
 diagnostic_aggregator
 pluginlib
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_analyzers
  CATKIN_DEPENDS 
    pluginlib
    roscpp
    diagnostic_msgs
    diagnostic_aggregator
)

add_compile_options(-std=c++11) 

include_directories(
 include/status_aggregator_ros
 ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ANALYZER_SRCS src/status_aggregator_ros/StatusAnalyzer.cpp)
add_library(${PROJECT_NAME}_analyzers ${ANALYZER_SRCS})
target_link_libraries(${PROJECT_NAME}_analyzers ${catkin_LIBRARIES})