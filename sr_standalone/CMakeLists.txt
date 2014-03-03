cmake_minimum_required(VERSION 2.8.3)
project(sr_standalone)
find_package(catkin REQUIRED COMPONENTS sr_hand)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS sr_hand
    INCLUDE_DIRS include
    LIBRARIES
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)
