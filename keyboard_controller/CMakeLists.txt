cmake_minimum_required(VERSION 3.0.2)

project(keyboard_controller)

find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
)

catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
		roscpp
		std_msgs
		geometry_msgs
	DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_controller_publisher src/keyboard_subscriber.cpp)

target_link_libraries(keyboard_controller_publisher ${catkin_LIBRARIES})
