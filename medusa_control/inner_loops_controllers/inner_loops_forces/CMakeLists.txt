cmake_minimum_required(VERSION 3.1)
project(inner_loops_forces)

find_package(catkin REQUIRED COMPONENTS
  medusa_msgs
  dsor_msgs
  roscpp
  auv_msgs
  medusa_gimmicks_library
  message_generation
)

add_compile_options(-std=c++17)

add_service_files(
  FILES
  ChangeInnerGains.srv
  ChangeInnerLimits.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp medusa_gimmicks_library auv_msgs)

include_directories(include include/inner_loops_forces_ros include/inner_loops_forces_controllers)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(inner_loops_forces src/inner_loops_forces_ros/innerloops_node.cpp src/inner_loops_forces_ros/innerloops.cpp src/inner_loops_forces_controllers/ros_controller.cpp src/inner_loops_forces_ros/thruster_allocation.cpp src/inner_loops_forces_controllers/pid_controller.cpp src/inner_loops_forces_ros/safeties.cpp)
add_dependencies(inner_loops_forces ${inner_loops_forces_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inner_loops_forces ${catkin_LIBRARIES})