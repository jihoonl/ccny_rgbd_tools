cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Generate services
rosbuild_gensrv()

# Generate dynamic parameters
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

################################################################
# Dependencies
################################################################

# OpenCV
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${OpenCV_LIBRARIES})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_LIBRARIES})


if(NOT ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "armv7l")

	# Octomap
	find_package(octomap REQUIRED)
	include_directories(${OCTOMAP_INCLUDE_DIRS})
	link_directories(${OCTOMAP_LIBRARY_DIRS})
	link_libraries(${OCTOMAP_LIBRARIES})


	#g2o
	set(G2O_LIBRARIES 
	  g2o_core 
	  g2o_stuff
	  g2o_solver_cholmod 
	  g2o_solver_csparse 
	  g2o_types_slam3d
	  cholmod)
	  

endif(NOT ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "armv7l")


# boost
rosbuild_add_boost_directories()

################################################################
# Build keyframe sender application
################################################################

rosbuild_add_executable(keyframe_sender
  src/node/keyframe_sender_node.cpp
  src/apps/keyframe_sender.cpp
  src/util.cpp)

rosbuild_add_library(keyframe_sender_nodelet
  src/nodelet/keyframe_sender_nodelet.cpp
  src/apps/keyframe_sender.cpp
  src/util.cpp)

target_link_libraries (keyframe_sender
  rgbdtools
  ${OpenCV_LIBRARIES})

target_link_libraries (keyframe_sender_nodelet
  rgbdtools
  ${OpenCV_LIBRARIES})
  
  

