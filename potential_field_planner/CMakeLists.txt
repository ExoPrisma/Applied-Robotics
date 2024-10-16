cmake_minimum_required(VERSION 3.10)
project(potential_field_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS    
		${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS    
  DEPENDS
		roscpp 
		std_msgs 
		sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS} 
)

add_executable(joint_planner src/joint_planner.cpp)

target_link_libraries(joint_planner
  ${catkin_LIBRARIES}
)
