cmake_minimum_required(VERSION 3.8)
project(map_server)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(globalmap_server_node globalmap_server_node.cpp)
ament_target_dependencies(globalmap_server_node
  rclcpp
  sensor_msgs
  std_msgs
  pcl_conversions
)
target_link_libraries(globalmap_server_node ${PCL_LIBRARIES})
target_include_directories(globalmap_server_node PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS globalmap_server_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
