cmake_minimum_required(VERSION 2.8.3)
project(sr_self_test)
find_package(catkin REQUIRED COMPONENTS rostest roscpp self_test sr_hand sr_movements sr_robot_msgs)
find_package(Boost REQUIRED COMPONENTS thread system iostreams filesystem)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp self_test sr_hand sr_movements sr_robot_msgs
    INCLUDE_DIRS include
    LIBRARIES sr_self_test
)

add_library(sr_self_test src/sr_self_test.cpp src/sr_test_runner.cpp src/test_joint_movement.cpp src/manual_test.cpp
  src/diagnostic_parser.cpp src/sensor_noise_test.cpp src/motor_test.cpp)
add_dependencies(sr_self_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_self_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


# if(COMMAND add_rostest_gtest)
#   add_rostest_gtest(sr_self_test_test test/sr_self_test.test test/test_self_test.cpp)
#   add_dependencies(sr_self_test_test ${catkin_EXPORTED_TARGETS})
#   target_link_libraries(sr_self_test_test  sr_self_test ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
# endif(COMMAND add_rostest_gtest)


install(TARGETS sr_self_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
