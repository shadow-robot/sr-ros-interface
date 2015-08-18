cmake_minimum_required(VERSION 2.8.3)
project(sr_gazebo_sim)

find_package(catkin REQUIRED COMPONENTS roscpp sr_hardware_interface cmake_modules ros_ethercat_model gazebo_ros_control)
find_package(TinyXML REQUIRED)
find_package(gazebo REQUIRED)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS roscpp sr_hardware_interface ros_ethercat_model gazebo_ros_control
    INCLUDE_DIRS include
    LIBRARIES sr_gazebo_sim
)

add_library(sr_gazebo_sim src/gazebo_hardware_sim.cpp)
add_dependencies(sr_gazebo_sim ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_gazebo_sim ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS sr_gazebo_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  OPTIONAL
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES gazebo_sim_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
