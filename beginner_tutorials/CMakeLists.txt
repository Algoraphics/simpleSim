cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_executable(talker src/talker.cpp)
rosbuild_add_executable(listener src/listener.cpp)
rosbuild_add_executable(steeringControl src/steeringControl.cpp)
rosbuild_add_executable(wrenchTalker src/wrenchTalker.cpp)
rosbuild_add_executable(talkerInt src/talkerInt.cpp)
rosbuild_add_executable(listenerInt src/listenerIntInt.cpp)

