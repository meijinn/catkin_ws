cmake_minimum_required(VERSION 3.0.2)
project(urg_gmapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib
  move_base_msgs
  roslib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(run_cpp src/run_cpp.cpp)
target_link_libraries(run_cpp ${catkin_LIBRARIES})