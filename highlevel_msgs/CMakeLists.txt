cmake_minimum_required(VERSION 3.10)
project(highlevel_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS        
    genmsg
)

# Declare ROS service
add_service_files(
	FILES 
	MoveTo.srv
	SetKD.srv
)

generate_messages(DEPENDENCIES )

catkin_package(
  INCLUDE_DIRS    
  LIBRARIES 
  CATKIN_DEPENDS    
  DEPENDS
 
)

#############
## Install ##
#############

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
