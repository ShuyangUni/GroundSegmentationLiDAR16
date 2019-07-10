cmake_minimum_required(VERSION 3.0.0)
project(MyGroundSeg VERSION 0.1.0)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 

add_executable(MyGroundSeg 
                    src/main.cpp
                    src/GroundSegmentor.cpp)
target_link_libraries(MyGroundSeg ${PCL_LIBRARIES})
