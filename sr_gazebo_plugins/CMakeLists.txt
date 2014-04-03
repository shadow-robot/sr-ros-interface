cmake_minimum_required(VERSION 2.8.3)
project(sr_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs sensor_msgs
  pr2_msgs geometry_msgs diagnostic_updater diagnostic_msgs angles pr2_hardware_interface sr_hardware_interface pr2_controller_manager
  pr2_mechanism_model urdf dynamic_reconfigure sr_hardware_interface sr_self_test cmake_modules)
find_package(TinyXML REQUIRED)
find_package(PkgConfig)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_definitions(-fPIC -DSIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE)


catkin_package(
    DEPENDS python-numpy opende TinyXML gazebo
    CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs sensor_msgs pr2_msgs geometry_msgs
      diagnostic_updater diagnostic_msgs angles pr2_hardware_interface sr_hardware_interface pr2_controller_manager urdf
      pr2_mechanism_model dynamic_reconfigure sr_hardware_interface sr_self_test
    INCLUDE_DIRS include
    LIBRARIES 
)

add_library(sr_gazebo_ros_controller_manager src/gazebo_ros_controller_manager.cpp)
add_dependencies(sr_gazebo_ros_controller_manager ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_gazebo_ros_controller_manager ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


install(TARGETS sr_gazebo_ros_controller_manager
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
