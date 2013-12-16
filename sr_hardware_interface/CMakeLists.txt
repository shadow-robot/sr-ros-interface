cmake_minimum_required(VERSION 2.8.3)
project(sr_hardware_interface)
find_package(catkin REQUIRED COMPONENTS roscpp pr2_hardware_interface)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp pr2_hardware_interface
    INCLUDE_DIRS include
    LIBRARIES
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)