# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_tactile_sensors)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sr_robot_msgs visualization_msgs gazebo_plugins)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#add_message_files(
  FILES
  # TODO: List your msg files here
)
#uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)


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
add_executable(sr_tactile_virtual src/sr_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
IF(gazebo_support)
  add_executable(sr_tactile_virtual_gazebo src/sr_gazebo_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
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
  add_executable(sr_tactile_real src/sr_real_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
  target_link_libraries(sr_tactile_real robot)
ENDIF(real_hand)

#############################
#  RVIZ MARKER FOR TACTILES #
#############################
add_executable(sr_tactile_rviz_marker src/sr_tactile_rviz_marker.cpp)
add_executable(sr_bumper_rviz_marker src/sr_bumper_rviz_marker.cpp)
###############
#   EXAMPLES  #
###############
add_executable(tactile_example examples/tactile.cpp)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp std_msgs sr_robot_msgs visualization_msgs gazebo_plugins
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)