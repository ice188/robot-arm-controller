
cmake_minimum_required(VERSION 3.10)
project(base_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    tf2
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS    
  	${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
  LIBRARIES 
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs tf_geometry_msgs tf2
  DEPENDS
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(bc_node src/baseController.cpp)
target_link_libraries(bc_node ${catkin_LIBRARIES})

install(
  TARGETS bc_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
