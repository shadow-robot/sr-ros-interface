cmake_minimum_required(VERSION 2.8.3)
project(sr_mechanism_model)
find_package(catkin REQUIRED COMPONENTS roscpp sr_hardware_interface cmake_modules ros_ethercat_model)
find_package(TinyXML REQUIRED)
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

catkin_package(
    DEPENDS TinyXML
    CATKIN_DEPENDS roscpp sr_hardware_interface ros_ethercat_model
    INCLUDE_DIRS include
    LIBRARIES sr_mechanism_model
)

add_library(sr_mechanism_model src/joint_0_transmission.cpp src/simple_transmission.cpp
  src/joint_0_transmission_for_muscle.cpp src/simple_transmission_for_muscle.cpp src/null_transmission.cpp)
add_dependencies(sr_mechanism_model ${catkin_EXPORTED_TARGETS})
target_link_libraries(sr_mechanism_model ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS sr_mechanism_model 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  OPTIONAL
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES transmission_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
