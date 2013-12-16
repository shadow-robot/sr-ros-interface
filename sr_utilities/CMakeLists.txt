cmake_minimum_required(VERSION 2.8.3)
project(sr_utilities)
find_package(catkin REQUIRED COMPONENTS sensor_msgs sr_robot_msgs rospy roscpp tf)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_service_files(FILES getJointState.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

add_library(sr_calibration src/calibration.cpp)
add_dependencies(sr_calibration sr_utilities_gencpp)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS sensor_msgs sr_robot_msgs rospy roscpp tf
    INCLUDE_DIRS include
    LIBRARIES
)

catkin_add_gtest(test_calibration test/test_calibration.cpp src/calibration.cpp)
add_dependencies(test_calibration sr_utilities_gencpp)
catkin_add_gtest(test_thread_safe_map test/test_thread_safe_map.cpp)
add_dependencies(test_thread_safe_map sr_utilities_gencpp)
catkin_add_gtest(test_math_utils test/test_math_utils.cpp)
add_dependencies(test_math_utils sr_utilities_gencpp)
target_link_libraries(test_math_utils pthread)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY scripts/
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   USE_SOURCE_PERMISSIONS
   PATTERN ".svn" EXCLUDE
)

install(TARGETS sr_calibration
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)