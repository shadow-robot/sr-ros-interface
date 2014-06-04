cmake_minimum_required(VERSION 2.8.3)

project(sr_standalone_example)

find_package(sr_standalone REQUIRED)

include_directories(include ${sr_standalone_INCLUDE_DIRS})
add_executable(standalone_example standalone_example.cpp)
target_link_libraries(standalone_example ${sr_standalone_LIBRARIES})

install(TARGETS standalone_example
   RUNTIME DESTINATION ./bin
)
