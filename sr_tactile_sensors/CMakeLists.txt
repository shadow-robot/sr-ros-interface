cmake_minimum_required(VERSION 2.8.3)
project(sr_tactile_sensors)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sr_robot_msgs visualization_msgs gazebo_plugins)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

set(ROS_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Gazebo
set(gazebo_support $ENV{GAZEBO})
message("--------- Shadow Robot Tactiles Package Configuration:")
if(gazebo_support)
  set( CMAKE_CXX_FLAGS "-D GAZEBO")
  message(" [x] Gazebo support")
else(gazebo_support)
  message(" [ ] Gazebo support")
endif(gazebo_support)

# Real Hand
set(real_hand $ENV{REAL_HAND})
if(real_hand)
  message(" [x] Real Hand.")
else(real_hand)
  message(" [ ] Real Hand.")
endif(real_hand)

#############################
#  VIRTUAL TACTILE SENSORS  #
#############################
add_executable(sr_tactile_virtual src/sr_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
add_dependencies(sr_tactile_virtual sr_robot_msgs_gencpp)
if(gazebo_support)
  add_executable(sr_tactile_virtual_gazebo src/sr_gazebo_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
  add_dependencies(sr_tactile_virtual_gazebo sr_robot_msgs_gencpp)
endif(gazebo_support)

#############################
#   REAL TACTILE SENSORS    #
#############################
# Uncomment the following lines if you want access to the real tactile sensors
if(real_hand)
  include_directories(/usr/realtime/include/)
  add_library(robot STATIC IMPORTED)
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  add_executable(sr_tactile_real src/sr_real_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
  add_dependencies(sr_tactile_real sr_robot_msgs_gencpp)
  target_link_libraries(sr_tactile_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} robot pthread)
endif(real_hand)

#############################
#  RVIZ MARKER FOR TACTILES #
#############################
add_executable(sr_tactile_rviz_marker src/sr_tactile_rviz_marker.cpp)
add_dependencies(sr_tactile_rviz_marker sr_robot_msgs_gencpp)
add_executable(sr_bumper_rviz_marker src/sr_bumper_rviz_marker.cpp)
add_dependencies(sr_bumper_rviz_marker sr_robot_msgs_gencpp)
###############
#   EXAMPLES  #
###############
add_executable(tactile_example examples/tactile.cpp)
add_dependencies(tactile_example sr_robot_msgs_gencpp)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp std_msgs sr_robot_msgs visualization_msgs gazebo_plugins
    INCLUDE_DIRS include
    LIBRARIES
)