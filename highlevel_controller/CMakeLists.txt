cmake_minimum_required(VERSION 3.10)
project(highlevel_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS        
    kinematic_controller
    control_msgs
)


catkin_package(
  INCLUDE_DIRS    
  LIBRARIES 
  CATKIN_DEPENDS  
  DEPENDS
    kinematic_controller
    control_msgs
)

include_directories(
	include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(action_client src/action_client.cpp)
target_link_libraries(action_client ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
