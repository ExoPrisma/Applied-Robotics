cmake_minimum_required(VERSION 3.10)
project(cubic_polynomial_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS        
    roscpp
    std_msgs
    highlevel_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS    
  LIBRARIES 
  CATKIN_DEPENDS    
  DEPENDS
    roscpp
    std_msgs
    highlevel_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS} 
)

add_executable(pose_planner src/pose_planner.cpp)
target_link_libraries(pose_planner ${catkin_LIBRARIES})
