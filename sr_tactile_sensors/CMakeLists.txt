cmake_minimum_required(VERSION 2.8.3)
project(sr_tactile_sensors)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sr_robot_msgs visualization_msgs gazebo_msgs gazebo_ros)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp std_msgs sr_robot_msgs visualization_msgs gazebo_msgs gazebo_ros
    INCLUDE_DIRS include
    LIBRARIES
)

# Real Hand
set(real_hand $ENV{REAL_HAND})
if(real_hand)
  message(" [x] Real Hand.")
else(real_hand)
  message(" [ ] Real Hand.")
endif(real_hand)

#############################
#  VIRTUAL TACTILE SENSORS  #
#############################
add_executable(sr_tactile_virtual src/sr_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
add_dependencies(sr_tactile_virtual src ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_tactile_virtual ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(sr_tactile_virtual_gazebo src/sr_gazebo_virtual_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
add_dependencies(sr_tactile_virtual_gazebo ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_tactile_virtual_gazebo ${Boost_LIBRARIES} ${catkin_LIBRARIES})


#############################
#   REAL TACTILE SENSORS    #
#############################
# Uncomment the following lines if you want access to the real tactile sensors
if(real_hand)
  include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
  include_directories(/usr/realtime/include/)
  add_library(robot STATIC IMPORTED)
  add_dependencies(robot ${catkin_EXPORTED_TARGETS})
  set_property(TARGET robot PROPERTY IMPORTED_LOCATION /usr/lib/robot.a)
  add_executable(sr_tactile_real src/sr_real_tactile_sensor.cpp src/sr_generic_tactile_sensor.cpp)
  add_dependencies(sr_tactile_real ${catkin_EXPORTED_TARGETS})
  target_link_libraries(sr_tactile_real ${Boost_LIBRARIES} ${catkin_LIBRARIES} robot)
endif(real_hand)

#############################
#  RVIZ MARKER FOR TACTILES #
#############################
add_executable(sr_tactile_rviz_marker src/sr_tactile_rviz_marker.cpp)
add_dependencies(sr_tactile_rviz_marker ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_tactile_rviz_marker ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_executable(sr_bumper_rviz_marker src/sr_bumper_rviz_marker.cpp)
add_dependencies(sr_bumper_rviz_marker ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_bumper_rviz_marker ${Boost_LIBRARIES} ${catkin_LIBRARIES})

###############
#   EXAMPLES  #
###############
add_executable(tactile_example examples/tactile.cpp)
add_dependencies(tactile_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(tactile_example ${Boost_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS sr_tactile_virtual sr_tactile_rviz_marker sr_bumper_rviz_marker tactile_example sr_tactile_virtual_gazebo
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY params/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
