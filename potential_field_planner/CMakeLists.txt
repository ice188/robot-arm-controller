cmake_minimum_required(VERSION 3.10)
project(potential_field_planner)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_srvs
    geometry_msgs
    std_msgs
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_srvs geometry_msgs std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(pfp_node
	src/potentialFieldPlanner.cpp 
)
target_link_libraries(pfp_node
	${catkin_LIBRARIES} 
)

## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS pfp_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


