cmake_minimum_required(VERSION 2.8.3)
project(sr_movements)
find_package(catkin REQUIRED COMPONENTS pr2_controllers_msgs sr_robot_msgs std_msgs sensor_msgs sr_utilities roscpp sr_hand rostest)
find_package(ImageMagick COMPONENTS Magick++ REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${ImageMagick_INCLUDE_DIRS})

add_executable(sr_movements src/sr_movements.cpp src/movement_from_image.cpp src/partial_movement.cpp src/movement_publisher.cpp)
add_dependencies(sr_movements ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_movements ${ImageMagick_LIBRARIES} ${catkin_LIBRARIES})

# sr_movements executable needs to be copied to a devel subfolder for the following test to run 
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/lib/sr_movements)
add_custom_command(TARGET sr_movements POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy sr_movements ${CATKIN_DEVEL_PREFIX}/lib/sr_movements)

add_rostest(test/test_movements.test)
add_rostest(test/test_joint_name_movement.test)

add_library(sr_movement src/movement_from_image.cpp src/partial_movement.cpp src/movement_publisher.cpp)
add_dependencies(sr_movement ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_movement ${ImageMagick_LIBRARIES} ${catkin_LIBRARIES})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS pr2_controllers_msgs sr_robot_msgs std_msgs sensor_msgs sr_utilities roscpp sr_hand 
    INCLUDE_DIRS include
    LIBRARIES sr_movement
)

install(TARGETS sr_movement sr_movements
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
