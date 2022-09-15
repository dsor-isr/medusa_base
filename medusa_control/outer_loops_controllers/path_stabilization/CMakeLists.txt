cmake_minimum_required(VERSION 3.1)
project(path_stabilization)

find_package(catkin 
  # ROS components
  REQUIRED COMPONENTS
  std_msgs 
  auv_msgs
  medusa_msgs
  roscpp
  message_generation
  waypoint

  # DSOR utils library
  dsor_utils

  # Medusa gimmicks library 
  medusa_gimmicks_library
)

find_package(
  # Eigen Library required
  Eigen3 REQUIRED  
)

# Include here your services for the Path Stabilization
add_service_files(
  FILES
  ResetVT.srv
  SetPS.srv
  StartPS.srv
  StopPS.srv
  UpdateGainsPS.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  
  # Include for message generation
  message_runtime
)

add_compile_options(-std=c++17 -Wall) 

include_directories(
  include/ros
  include/algorithms
  ${catkin_INCLUDE_DIRS}
  # Eigen Library is required for this package
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node 
  
  # ROS specific details 
  src/ros/PathStabilizationNode.cpp
  src/ros/PathStabilizationServices.cpp
	
  # Path Stabilization Algorithms Implementations
  src/algorithms/PathStabilization.cpp
  src/algorithms/Potes.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)
