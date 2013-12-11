# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_gazebo_plugins)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS gazebo gazebo_plugins gazebo_tools opende roscpp rospy std_msgs nav_msgs sensor_msgs cv_bridge pr2_msgs geometry_msgs opencv2 diagnostic_updater cv_bridge diagnostic_msgs angles pr2_hardware_interface sr_hardware_interface pr2_controller_manager pr2_mechanism_model bullet urdf polled_camera image_transport dynamic_reconfigure sr_hardware_interface pr2_gazebo_plugins sr_self_test python-numpy tinyxml)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDeb)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(
  FILES
  # TODO: List your msg files here
)
add_service_files(
  FILES
  # TODO: List your msg files here
)

add_definitions(-fPIC)

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories( ${ImageMagick_INCLUDE_DIRS})


#
# if screw joint exists, remove hack for damping variable
#
execute_process(COMMAND rosrun opende ode-config --version OUTPUT_VARIABLE ode_version)
if (${ode_version} VERSION_LESS "0.11.1.71")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${ode_version} no screw joint")
else (${ode_version} VERSION_LESS "0.11.1.71")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${ode_version} screw joint exists, removing hack")
  add_definitions(-DODE_SCREW_JOINT)
endif (${ode_version} VERSION_LESS "0.11.1.71")




# dynamic reconfigure for gazebo ros camera nodes
# does not exist until simulator_gazebo 1.2.9
execute_process(COMMAND rosversion simulator_gazebo OUTPUT_VARIABLE simulator_gazebo_version)
if (${simulator_gazebo_version} VERSION_GREATER "1.2.8")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${simulator_gazebo_version}Enabling gazebo ros camera dynamic reconfigure")
  add_definitions(-DSIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE)
else (${simulator_gazebo_version} VERSION_GREATER "1.2.8")
  MESSAGE(STATUS "OUTPUT_VARIABLE: ${simulator_gazebo_version}Disabling gazebo ros camera dynamic reconfigure")
endif (${simulator_gazebo_version} VERSION_GREATER "1.2.8")


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()

add_library(sr_gazebo_ros_controller_manager src/gazebo_ros_controller_manager.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(sr_gazebo_ros_controller_manager ${Boost_LIBRARIES})

exec_program("glxinfo" OUTPUT_VARIABLE glx_out RETURN_VALUE glx_val)
if(${glx_val} EQUAL 0)
  message(STATUS "run test here")
else(${glx_val} EQUAL 0)
  message(STATUS "no test here")
endif(${glx_val} EQUAL 0)

## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS gazebo gazebo_plugins gazebo_tools opende roscpp rospy std_msgs nav_msgs sensor_msgs cv_bridge pr2_msgs geometry_msgs opencv2 diagnostic_updater cv_bridge diagnostic_msgs angles pr2_hardware_interface sr_hardware_interface pr2_controller_manager pr2_mechanism_model bullet urdf polled_camera image_transport dynamic_reconfigure sr_hardware_interface pr2_gazebo_plugins sr_self_test python-numpy tinyxml
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)