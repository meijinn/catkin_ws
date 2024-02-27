cmake_minimum_required(VERSION 3.0.2)
project(slam_dungeon)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
  rospy
  std_msgs
  tf
  move_base_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  roslib
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(run_cpp src/run_cpp.cpp)
target_link_libraries(run_cpp ${catkin_LIBRARIES})

add_executable(info_float32_publisher src/info_float32_publisher.cpp)
target_link_libraries(info_float32_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(info_overlay_menu_publisher src/info_overlay_menu_publisher.cpp)
target_link_libraries(info_overlay_menu_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(info_overlay_text_publisher src/info_overlay_text_publisher.cpp)
target_link_libraries(info_overlay_text_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(goal_status src/goal_status.cpp)
target_link_libraries(goal_status
  ${catkin_LIBRARIES}
)

add_executable(odometry_listener src/odometry_listener.cpp)
target_link_libraries(odometry_listener
  ${catkin_LIBRARIES}
)

add_executable(odometry_talker src/odometry_talker.cpp)
target_link_libraries(odometry_talker
  ${catkin_LIBRARIES}
)