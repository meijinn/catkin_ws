cmake_minimum_required(VERSION 3.0.2)
project(g29mt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(ps3_pulse src/ps3_pulse.cpp)

target_link_libraries(ps3_pulse
  ${catkin_LIBRARIES}
)

add_executable(ps3_pca9685 src/ps3_pca9685.cpp)

target_link_libraries(ps3_pca9685
  ${catkin_LIBRARIES}
)

add_executable(ge_force_pca9685_taikan src/ge_force_pca9685_taikan.cpp)

target_link_libraries(ge_force_pca9685_taikan
  ${catkin_LIBRARIES}
)

add_executable(ps3_twist_pub src/ps3_twist_pub.cpp)

target_link_libraries(ps3_twist_pub
  ${catkin_LIBRARIES}
)

add_executable(g29_twist_pub src/g29_twist_pub.cpp)

target_link_libraries(g29_twist_pub
  ${catkin_LIBRARIES}
)