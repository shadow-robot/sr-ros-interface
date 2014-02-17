cmake_minimum_required(VERSION 2.8.3)
project(sr_kinematics)
find_package(catkin REQUIRED COMPONENTS roscpp tf urdf kdl_parser sensor_msgs tf_conversions moveit_core)
find_package(orocos_kdl REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

catkin_package(
    DEPENDS orocos_kdl
    CATKIN_DEPENDS roscpp tf urdf kdl_parser sensor_msgs tf_conversions moveit_core
    INCLUDE_DIRS
    LIBRARIES
)

add_executable(sr_kinematics src/sr_kinematics.cpp)
target_link_libraries(sr_kinematics ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS sr_kinematics
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
