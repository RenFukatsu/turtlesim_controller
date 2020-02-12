cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_controller)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  roscpp
  turtlesim
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlesim_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtlesim_controller src/turtlesim_controller.cpp)
add_dependencies(turtlesim_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlesim_controller
  ${catkin_LIBRARIES}
)
