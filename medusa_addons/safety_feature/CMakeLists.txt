cmake_minimum_required(VERSION 3.1)
project(safety_feature)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  medusa_msgs
)


catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(safety_feature src/Safety_Feature.cpp)
add_executable(wifi_acoustic_safety src/wifi_acoustic_safety.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(safety_feature ${catkin_EXPORTED_TARGETS})
add_dependencies(wifi_acoustic_safety ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(safety_feature
  ${catkin_LIBRARIES}
)

target_link_libraries(wifi_acoustic_safety
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############

install(PROGRAMS
  src/Client/SafetyFeatureCLT
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(TARGETS wifi_acoustic_safety safety_feature
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)