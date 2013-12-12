cmake_minimum_required(VERSION 2.8.3)
project(sr_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs sensor_msgs gazebo_plugins gazebo_ros gazebo_ros_control
  pr2_msgs geometry_msgs diagnostic_updater diagnostic_msgs angles pr2_hardware_interface sr_hardware_interface pr2_controller_manager
  pr2_mechanism_model urdf dynamic_reconfigure sr_hardware_interface sr_self_test)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

set(ROS_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_definitions(-fPIC)

#find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
#include_directories( ${ImageMagick_INCLUDE_DIRS})

# if screw joint exists, remove hack for damping variable
execute_process(COMMAND ode-config --version OUTPUT_VARIABLE ode_version)
if (${ode_version} VERSION_LESS "0.11.1.71")
  message(STATUS "OUTPUT_VARIABLE: ${ode_version} no screw joint")
else (${ode_version} VERSION_LESS "0.11.1.71")
  message(STATUS "OUTPUT_VARIABLE: ${ode_version} screw joint exists, removing hack")
  add_definitions(-DODE_SCREW_JOINT)
endif (${ode_version} VERSION_LESS "0.11.1.71")

# dynamic reconfigure for gazebo ros camera nodes
# does not exist until simulator_gazebo 1.2.9
#execute_process(COMMAND rosversion simulator_gazebo OUTPUT_VARIABLE simulator_gazebo_version)
#if (${simulator_gazebo_version} VERSION_GREATER "1.2.8")
#  message(STATUS "OUTPUT_VARIABLE: ${simulator_gazebo_version}Enabling gazebo ros camera dynamic reconfigure")
#  add_definitions(-DSIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE)
#else (${simulator_gazebo_version} VERSION_GREATER "1.2.8")
#  message(STATUS "OUTPUT_VARIABLE: ${simulator_gazebo_version}Disabling gazebo ros camera dynamic reconfigure")
#endif (${simulator_gazebo_version} VERSION_GREATER "1.2.8")

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
add_library(sr_gazebo_ros_controller_manager src/gazebo_ros_controller_manager.cpp)
target_link_libraries(sr_gazebo_ros_controller_manager ${Boost_LIBRARIES} ${catkin_LIBRARIES})

execute_process(COMMAND glxinfo OUTPUT_VARIABLE glx_out)
if(${glx_out} EQUAL 0)
  message(STATUS "run test here")
else(${glx_out} EQUAL 0)
  message(STATUS "no test here")
endif(${glx_out} EQUAL 0)

catkin_package(
    DEPENDS python-numpy opende opencv2 bullet
    CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs pr2_msgs geometry_msgs gazebo_plugins gazebo_ros gazebo_ros_control
      diagnostic_updater diagnostic_msgs angles pr2_hardware_interface sr_hardware_interface pr2_controller_manager urdf
      pr2_mechanism_model polled_camera image_transport dynamic_reconfigure sr_hardware_interface sr_self_test 
    INCLUDE_DIRS include
    LIBRARIES
)