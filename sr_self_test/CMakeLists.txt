# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_self_test)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp self_test sr_hand sr_movements sr_robot_msgs)

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

#uncomment if you have defined messages
#add_message_files(
  FILES
  # TODO: List your msg files here
)
#uncomment if you have defined services
#add_service_files(
  FILES
  # TODO: List your msg files here
)

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories( ${ImageMagick_INCLUDE_DIRS})

#common commands for building c++ executables and libraries
#add_library(gnuplot_cpp src/gnuplot.cpp)
add_library(sr_self_test src/sr_self_test.cpp src/sr_test_runner.cpp src/test_joint_movement.cpp src/manual_test.cpp src/diagnostic_parser.cpp src/sensor_noise_test.cpp src/motor_test.cpp)
find_package(Boost REQUIRED COMPONENTS thread system iostreams filesystem)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(sr_self_test ${Boost_LIBRARIES})
target_link_libraries(sr_self_test ${Boost_LIBRARIES})
target_link_libraries(sr_self_test ${Boost_LIBRARIES})
target_link_libraries(sr_self_test ${Boost_LIBRARIES})

#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#target_link_libraries(example ${PROJECT_NAME})

###
# Tests
###
add_executable(sr_self_test_test test/test_self_test.cpp)

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_gtest_build_flags(sr_self_test)
target_link_libraries(sr_self_test_test sr_self_test)
add_rostest(test/sr_self_test.test)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp self_test sr_hand sr_movements sr_robot_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)