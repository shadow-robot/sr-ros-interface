# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_utilities)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS sensor_msgs sr_robot_msgs rospy roscpp tf)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_service_files(
  FILES
  # TODO: List your msg files here
)

add_library(sr_calibration src/calibration.cpp)

catkin_add_gtest(test/test_calibration test/test_calibration.cpp src/calibration.cpp)

SET(jenkins $ENV{JENKINS})
IF(DEFINED jenkins)
  add_library(gcov STATIC IMPORTED)
  set_property(TARGET gcov PROPERTY IMPORTED_LOCATION /usr/lib/gcc/i486-linux-gnu/4.4/libgcov.a)
ENDIF(DEFINED jenkins)

catkin_add_gtest(test/test_thread_safe_map test/test_thread_safe_map.cpp)
IF(DEFINED jenkins)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
#   rosbuild_add_compile_flags(test/test_thread_safe_map -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_thread_safe_map gcov)
ENDIF(DEFINED jenkins)

catkin_add_gtest(test/test_math_utils test/test_math_utils.cpp)
IF(DEFINED jenkins)

# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
#   rosbuild_add_compile_flags(test/test_math_utils  -g -pg -fprofile-arcs -ftest-coverage)
  target_link_libraries(test/test_math_utils gcov)
ENDIF(DEFINED jenkins)
target_link_libraries(test/test_math_utils pthread)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS sensor_msgs sr_robot_msgs rospy roscpp tf
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)