cmake_minimum_required(VERSION 2.8.3)
project(mav_controller)
# add_definitions(-std=c99)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	cmake_modules
	roscpp
	std_msgs
	gazebo_msgs
	roslib
	hector_uav_msgs
	# message_generation
)

find_package(Eigen REQUIRED)

## Must be added in this order to successfully generate messages and build
# add_message_files(
#   FILES
#   RelativeData.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Declare Package ##
catkin_package(
	INCLUDE_DIRS
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp std_msgs gazebo_msgs roslib
 	DEPENDS Eigen
 	# message_runtime
)

## Directories to include
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIR}
)


## Library classes
add_library(paparazzi
	src/functions/coordinateconversions.c
	src/functions/randomgenerator.c
	src/functions/PID.c
	src/functions/collisioncone.c
	src/functions/discreteekf.c
	src/functions/fmatrix.c
	src/functions/shape.c
	src/functions/arrayfunctions.c
)

add_library(trackers 
	src/trackers/gazebotracker.cpp 
	src/trackers/statetracker.cpp 
	src/trackers/relativetracker.cpp
)

add_library(BTsimulator 
	src/trackers/BTsimulator.cpp
)

add_library(logger 
	src/loggers/logger.cpp
)


## Declare a C++ executable (i.e a node)
add_executable(behaviour src/behaviour.cpp)

## Link the node to the libraries needed
target_link_libraries(behaviour
	${catkin_LIBRARIES}
	${Eigen_LIBRARIES}
	trackers
	BTsimulator
	paparazzi
	logger
)

# Add dependencies to the node
add_dependencies(behaviour ${catkin_EXPORTED_TARGETS})
