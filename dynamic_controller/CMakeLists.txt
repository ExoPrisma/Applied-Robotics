cmake_minimum_required(VERSION 3.10)
project(dynamic_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS        
    roscpp
    std_msgs
		sensor_msgs
		highlevel_msgs
		potential_field_planner
		cubic_polynomial_planner
)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

catkin_package(
  INCLUDE_DIRS    
		${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS    
  DEPENDS
    roscpp
    std_msgs
		sensor_msgs
		highlevel_msgs
		potential_field_planner
		cubic_polynomial_planner
)

include_directories(
	include 
  ${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS} 
  ${PINOCCHIO_INCLUDE_DIRS}
)

add_executable(joint_controller src/joint_controller.cpp)
target_link_libraries(joint_controller ${catkin_LIBRARIES} ${PINOCCHIO_LIBRARIES})

