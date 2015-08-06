cmake_minimum_required(VERSION 2.8.3)
project(sr_utilities)
find_package(catkin REQUIRED COMPONENTS sensor_msgs sr_robot_msgs rospy roscpp tf)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(rostest REQUIRED)

catkin_python_setup()

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_service_files(FILES getJointState.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS sensor_msgs sr_robot_msgs rospy roscpp tf
    INCLUDE_DIRS include
    LIBRARIES sr_calibration
)

add_library(sr_calibration src/calibration.cpp)
add_dependencies(sr_calibration sr_utilities_gencpp ${catkin_EXPORTED_TARGETS})
add_library(sr_hand_finder src/sr_hand_finder.cpp)

catkin_add_gtest(test_calibration test/test_calibration.cpp src/calibration.cpp)
target_link_libraries(test_calibration ${catkin_LIBRARIES})
add_dependencies(test_calibration sr_utilities_gencpp ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(test_thread_safe_map test/test_thread_safe_map.cpp)
add_dependencies(test_thread_safe_map sr_utilities_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_thread_safe_map ${catkin_LIBRARIES} pthread)

catkin_add_gtest(test_math_utils test/test_math_utils.cpp)
add_dependencies(test_math_utils sr_utilities_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_math_utils ${catkin_LIBRARIES} pthread)
if(CATKIN_ENABLE_TESTING)
  add_rostest_gtest(test_hand_finder test/test_hand_finder.test test/test_hand_finder.cpp)
  add_dependencies(test_hand_finder sr_utilities_gencpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(test_hand_finder sr_hand_finder ${catkin_LIBRARIES})
endif()
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
)

install(TARGETS sr_calibration
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(PROGRAMS nodes/temperature_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS)
