cmake_minimum_required(VERSION 2.8.3)
project(sr_mechanism_model)
find_package(catkin REQUIRED COMPONENTS roscpp sr_hardware_interface pr2_mechanism_model)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(sr_mechanism_model src/joint_0_transmission.cpp src/simple_transmission.cpp
  src/joint_0_transmission_for_muscle.cpp src/simple_transmission_for_muscle.cpp)
target_link_libraries(sr_mechanism_model tinyxml ${catkin_LIBRARIES})

catkin_package(
    DEPENDS tinyxml
    CATKIN_DEPENDS roscpp sr_hardware_interface pr2_mechanism_model
    INCLUDE_DIRS include
    LIBRARIES
)