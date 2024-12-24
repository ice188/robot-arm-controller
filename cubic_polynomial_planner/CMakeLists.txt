cmake_minimum_required(VERSION 3.10)
project(cubic_polynomial_planner)

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
)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30") 


catkin_package(
	INCLUDE_DIRS
    #	include  
    	${Eigen_INCLUDE_DIRS}
		${PINOCCHIO_INCLUDE_DIRS}
  	LIBRARIES 
	  	pinocchio    	  		
  	CATKIN_DEPENDS
    	roscpp
		roslib
    	geometry_msgs 
        std_msgs   
		actionlib
		highlevel_msgs	
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
	${PINOCCHIO_INCLUDE_DIRS}
)


add_executable(server
	src/Server.cpp
)
target_link_libraries(server
  	${catkin_LIBRARIES}
	${Eigen_INCLUDE_DIRS}		
	${pinocchio_LIBRARIES} 	 
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

