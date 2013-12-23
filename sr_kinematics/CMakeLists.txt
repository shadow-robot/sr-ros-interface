cmake_minimum_required(VERSION 2.8.3)
project(sr_kinematics)
find_package(catkin REQUIRED COMPONENTS roscpp tf urdf kdl_parser sensor_msgs tf_conversions moveit_core orocos_kdl)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(sr_kinematics src/sr_kinematics.cpp)
target_link_libraries(sr_kinematics ${catkin_LIBRARIES})
catkin_package(
    DEPENDS orocos_kdl
    CATKIN_DEPENDS roscpp tf urdf kdl_parser sensor_msgs tf_conversions moveit_core
    INCLUDE_DIRS
    LIBRARIES
)
