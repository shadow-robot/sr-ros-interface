cmake_minimum_required(VERSION 2.8.3)
project(sr_standalone)
find_package(catkin REQUIRED COMPONENTS
    sr_hand
    sr_robot_msgs
    sensor_msgs
    std_msgs
    pr2_mechanism_msgs
    roscpp
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
        sr_hand
        sr_robot_msgs
        sensor_msgs
        std_msgs
        pr2_mechanism_msgs
        roscpp
    INCLUDE_DIRS include
    LIBRARIES sr_standalone
)

add_library(sr_standalone src/shadow_hand.cpp src/sr_ros_wrapper.cpp)
target_link_libraries(sr_standalone ${catkin_LIBRARIES})
add_dependencies(sr_standalone ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS sr_standalone
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
