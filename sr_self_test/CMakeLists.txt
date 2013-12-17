cmake_minimum_required(VERSION 2.8.3)
project(sr_self_test)
find_package(catkin REQUIRED COMPONENTS rostest roscpp self_test sr_hand sr_movements sr_robot_msgs)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${ImageMagick_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp self_test sr_hand sr_movements sr_robot_msgs
    INCLUDE_DIRS include
    LIBRARIES
)

if(COMMAND add_rostest_gtest)
  find_package(Boost REQUIRED COMPONENTS thread system iostreams filesystem)
  include_directories(${Boost_INCLUDE_DIRS} ${ImageMagick_INCLUDE_DIRS})
 
  add_library(sr_self_test src/sr_self_test.cpp src/sr_test_runner.cpp src/test_joint_movement.cpp src/manual_test.cpp
    src/diagnostic_parser.cpp src/sensor_noise_test.cpp src/motor_test.cpp)
  target_link_libraries(sr_self_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${ImageMagick_LIBRARIES} HandCommander sr_movement)

  add_rostest_gtest(sr_self_test_test test/sr_self_test.test test/test_self_test.cpp)
  target_link_libraries(sr_self_test_test sr_self_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${ImageMagick_LIBRARIES} HandCommander sr_movement)
endif(COMMAND add_rostest_gtest)