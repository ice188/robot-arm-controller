cmake_minimum_required(VERSION 3.10)
project(keyboard_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
add_executable (exb src/main.cpp)

include_directories (
        ${catkin_INCLUDE_DIRS}
)
# link the dependencies to the executable (roscpp and std_msgs)
target_link_libraries ( exb ${catkin_LIBRARIES} )
