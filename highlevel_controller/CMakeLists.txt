cmake_minimum_required(VERSION 3.10)
project(highlevel_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin 
	REQUIRED
	COMPONENTS
		roscpp
		geometry_msgs
    	std_msgs
		roslib
    	actionlib
		highlevel_msgs
		control_msgs
)
find_package(Eigen3 REQUIRED)

add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 
catkin_package(
	INCLUDE_DIRS
    #	include  
    	${Eigen_INCLUDE_DIRS}
  	LIBRARIES    	  		
  	CATKIN_DEPENDS
    	roscpp 
		actionlib
		highlevel_msgs	 
		control_msgs
  	DEPENDS
  	    	
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	#include
	${catkin_INCLUDE_DIRS}	
	${Eigen_INCLUDE_DIRS}		
)


add_executable(client
	src/Client.cpp
)
target_link_libraries(client
  	${catkin_LIBRARIES}
	${Eigen_INCLUDE_DIRS}
)


add_executable(gripper_client
	src/GripperClient.cpp
)
target_link_libraries(gripper_client
  	${catkin_LIBRARIES}
	${Eigen_INCLUDE_DIRS}
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

