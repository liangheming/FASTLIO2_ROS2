cmake_minimum_required(VERSION 3.26)
project(manual_initialization)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wno-unused-parameter -Wno-sign-compare -Wno-address)
  # Work around PCL 1.12 internal compiler error with GCC 11
  add_compile_options(-O2 -fno-tree-pre)
endif()

add_definitions(-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ndt_omp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(glog REQUIRED)

# PyTorch dependency for TurboReg
set(CMAKE_PREFIX_PATH "/usr/local;${CMAKE_PREFIX_PATH}")
#find_package(Torch REQUIRED)

if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# TODO: Add custom livox messages (if needed for ROS2)
# add_subdirectory(include/3rd/livox_ros_driver)

include_directories(
  include
  ${pcl_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
)

# Manual initialization node
add_executable(manual_initialization_node
  manual_initialization_node.cpp
  src/registration/ndt_registration.cpp
  src/registration/gicp_registration.cpp)
target_link_libraries(manual_initialization_node ${pcl_LIBRARIES})
ament_target_dependencies(manual_initialization_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  ndt_omp
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

install(TARGETS
  #funny_lidar_slam_node
  #global_localization_node
  manual_initialization_node
  #ros2_serial
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

ament_package()
