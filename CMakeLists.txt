cmake_minimum_required(VERSION 3.0.2)
project(fastslam20)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-g -Wall -lnlopt -lm -O3)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#add_compile_options(-std=c++11 -O3 -Wall -g) #-pg )

find_package(catkin REQUIRED COMPONENTS
  lfs_msgs
  roslib
  roscpp
  rospy
  std_msgs
  velocity_estimation
)

find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
#  LIBRARIES fastslam20
  CATKIN_DEPENDS roscpp rospy std_msgs roslib lfs_msgs velocity_estimation
#  DEPENDS system_lib
)
## Define common resources.
set (FASTSLAM_RESOURCES
  src/utils.cpp
  src/landmark.cpp
  src/particle.cpp
  src/loop_closing.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/
  ${EIGEN3_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS}
)
add_executable(slamming2 src/main.cpp ${FASTSLAM_RESOURCES})
add_dependencies(slamming2 lfs_msgs_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#add_dependencies(slamming2 velocity_estimation_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(slamming2 ${catkin_LIBRARIES} yaml-cpp nlopt)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
