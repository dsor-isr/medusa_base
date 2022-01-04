cmake_minimum_required(VERSION 3.1)
project(medusa_diagnostics_library)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 diagnostic_msgs
 std_msgs
)

add_compile_options(-std=c++17 -Wall -O3) 

catkin_package(
 INCLUDE_DIRS 
   include
   include/medusa_diagnostics_library
 LIBRARIES
  ${PROJECT_NAME}
 CATKIN_DEPENDS
  roscpp
  diagnostic_msgs
  std_msgs
)

include_directories(
 include
 include/medusa_diagnostics_library
 ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
   src/MedusaDiagnostics.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#add_definitions(-std=c++11)
#set(CMAKE_CXX_STANDARD 11) # C++11...
#set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
#set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11
#add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=0)
