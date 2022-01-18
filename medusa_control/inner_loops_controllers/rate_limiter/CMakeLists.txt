cmake_minimum_required(VERSION 3.1)
project(rate_limiter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_compile_options(-std=c++17 -Wall -O3)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} 
  src/ros/innerloops_node.cpp 
  src/ros/innerloops.cpp 
  src/controllers/ros_controller.cpp 
  src/controllers/pid_controller.cpp 
  src/ros/safeties.cpp)
add_dependencies(${PROJECT_NAME} ${rate_limiter_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)