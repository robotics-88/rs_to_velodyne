cmake_minimum_required(VERSION 3.5)
project(rs_to_velodyne)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(PCL REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
	include
	${PCL_INCLUDE_DIRS}
)

add_executable(rs_to_velodyne 
  src/rs_to_velodyne_node.cpp
)

add_library(rs_to_velodyne_lib SHARED
  src/rs_to_velodyne.cpp
)

ament_target_dependencies(rs_to_velodyne_lib 
  rclcpp
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
)


target_link_libraries(rs_to_velodyne 
  rs_to_velodyne_lib
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(rs_to_velodyne 
  rclcpp
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
)

#############
## Install ##
#############

install(TARGETS
  rs_to_velodyne
  rs_to_velodyne_lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(rs_to_velodyne)

ament_package()
