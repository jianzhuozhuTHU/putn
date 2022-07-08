cmake_minimum_required(VERSION 2.8.3)
project(gpr)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS   
  roscpp
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(gpr_path src/gpr_path.cpp)
target_link_libraries(gpr_path ${catkin_LIBRARIES})





