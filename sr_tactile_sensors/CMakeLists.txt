cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#Gazebo
SET(gazebo_support $ENV{GAZEBO})
MESSAGE("--------- Shadow Robot Tactiles Package Configuration:")
IF(gazebo_support)
  SET( CMAKE_CXX_FLAGS "-D GAZEBO")
  MESSAGE(" [x] Gazebo support")
ELSE(gazebo_support)
  MESSAGE(" [ ] Gazebo support")
ENDIF(gazebo_support)

# Real Hand
SET(real_hand $ENV{REAL_HAND})
IF(real_hand)
  MESSAGE(" [x] Real Hand.")
ELSE(real_hand)
  MESSAGE(" [ ] Real Hand.")
ENDIF(real_hand)

#############################
#  VIRTUAL TACTILE SENSORS  #
#############################
rosbuild_add_executable(sr_tactile_virtual src/sr_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
IF(gazebo_support)
  rosbuild_add_executable(sr_tactile_virtual_gazebo src/sr_gazebo_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
ENDIF(gazebo_support)


#############################
#   REAL TACTILE SENSORS    #
#############################
# Uncomment the following lines if you want access to the real tactile sensors
IF(real_hand)
  include_directories(/usr/realtime/include/)
  LINK_LIBRARIES(pthread)
  add_library(robot STATIC IMPORTED)
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  rosbuild_add_executable(sr_tactile_real src/sr_real_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
  target_link_libraries(sr_tactile_real robot)
ENDIF(real_hand)

#############################
#  RVIZ MARKER FOR TACTILES #
#############################
rosbuild_add_executable(sr_tactile_rviz_marker src/sr_tactile_rviz_marker.cpp)
rosbuild_add_executable(sr_bumper_rviz_marker src/sr_bumper_rviz_marker.cpp)
###############
#   EXAMPLES  #
###############
rosbuild_add_executable(tactile_example examples/tactile.cpp)
