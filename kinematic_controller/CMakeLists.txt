cmake_minimum_required(VERSION 3.10)
project(kinematic_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(catkin 
	REQUIRED
	COMPONENTS
		roscpp
		geometry_msgs
        std_msgs
		highlevel_msgs
		roslib
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
    	geometry_msgs 
        std_msgs   
		highlevel_msgs	 
		roslib
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


add_executable(joint_kinematic_controller
	src/jointController.cpp	
)
target_link_libraries(joint_kinematic_controller
  	${catkin_LIBRARIES} 
	
)


add_executable(inverse_kinematic_controller
	src/taskController.cpp
)
target_link_libraries(inverse_kinematic_controller
  	${catkin_LIBRARIES}
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

