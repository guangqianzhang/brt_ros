cmake_minimum_required(VERSION 3.0.2)
project(vision_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -fopenmp")

SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fopenmp")
set(cv_bridge_DIR /home/brt/Documents/cv_/devel/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  image_transport
  nodelet
  roscpp
  rospy
  sensor_msgs
  tf
  rockauto_msgs
  pcl_conversions
  pcl_ros
)

set(ALL_TARGET_LIBRARIES "")

include(../cmake/glog.cmake)
include(../cmake/YAML.cmake)
include(../cmake/OPENCV.cmake)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES version_camera
#  CATKIN_DEPENDS cv_bridge message_generation nodelet roscpp rospy sensor_msgs tf rockauto_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS} )
## Declare a C++ library
add_library(projection
  src/nodelet1.cpp 

)
add_dependencies(projection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(projection
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)
add_library(detection
  src/nodeletdetection.cpp  src/Yolo.cpp
  
)
add_dependencies(detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(detection
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/vision_camera_node.cpp ${ALL_SRCS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${ALL_TARGET_LIBRARIES}
)


