cmake_minimum_required(VERSION 2.8.3)
project(lidar_odom)

add_compile_options(-std=c++14 -li2c)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  nodelet
  cv_bridge
  image_transport
  tf2
  tf2_eigen
  tf2_ros
  ddynamic_reconfigure
  diagnostic_updater
  genmsg
  )


find_package(Eigen3 REQUIRED)


## Declare ROS messages and services
#add_message_files(FILES Num.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

catkin_package(INCLUDE_DIRS include)


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(lidar_odom_node src/pose.cpp)
target_link_libraries(lidar_odom_node ${catkin_LIBRARIES})
add_dependencies(lidar_odom_node beginner_tutorials_generate_messages_cpp)

install(TARGETS lidar_odom_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)