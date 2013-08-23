# Author: Ethan Rabb, based on a file by Sean Whitsitt
# Starting from scratch since we're having trouble 
# compiling on other computers.
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)



set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_BUILD_TYPE Debug)

project("ROS2JAUS")
cmake_minimum_required(VERSION 2.6)

include (../../src/cmake/FindOpenJaus.cmake)

#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg)

rosbuild_init()

enable_testing()

#find_package (ROS REQUIRED)

if(CYGWIN)
	set(CMAKE_CXX_FLAGS "-Wall -Di386 -Dlinux")
	set(CMAKE_C_FLAGS "-Wall -Di386 -Dlinux")
elseif(UNIX)
	set(CMAKE_CXX_FLAGS "-Wall -Di386")
	set(CMAKE_C_FLAGS "-Wall -Di386")
elseif(WIN32)
	set(CMAKE_CXX_FLAGS "-Wall -Di386")
	set(CMAKE_C_FLAGS "-Wall -Di386")
endif(CYGWIN)

add_subdirectory(libazcar)
add_subdirectory(libtutorial)
add_subdirectory(ojVehicleSim)
add_subdirectory(ojTutorial)
add_subdirectory(oj2torial)
add_subdirectory(ojROSComfort)
add_subdirectory(ojGazeboBridge)


