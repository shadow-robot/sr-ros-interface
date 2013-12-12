cmake_minimum_required(VERSION 2.8.3)
project(sr_movements)
find_package(catkin REQUIRED COMPONENTS pr2_controllers_msgs sr_robot_msgs std_msgs sensor_msgs sr_utilities roscpp sr_hand)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

set(ROS_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
#include_directories( ${ImageMagick_INCLUDE_DIRS})

add_executable(sr_movements src/sr_movements.cpp src/movement_from_image.cpp src/partial_movement.cpp src/movement_publisher.cpp)
add_dependencies(sr_movements sr_robot_msgs_gencpp)
target_link_libraries(sr_movements ${ImageMagick_LIBRARIES} ${catkin_LIBRARIES})

add_rostest(test/test_movements.test)
add_rostest(test/test_joint_name_movement.test)

add_library(sr_movement src/movement_from_image.cpp src/partial_movement.cpp src/movement_publisher.cpp)
add_dependencies(sr_movement sr_robot_msgs_gencpp libmagick)
target_link_libraries(sr_movement ${catkin_LIBRARIES} libmagick)

catkin_package(
    DEPENDS libmagick
    CATKIN_DEPENDS pr2_controllers_msgs sr_robot_msgs std_msgs sensor_msgs sr_utilities roscpp sr_hand 
    INCLUDE_DIRS include
    LIBRARIES
)