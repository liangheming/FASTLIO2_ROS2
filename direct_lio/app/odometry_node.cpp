#include "dlio/direct_odometry.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <glog/logging.h>

int main(int argc, char* argv[])
{
  // Initialize glog
  google::InitGoogleLogging(argv[0]);

  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<dlio::OdomNode>();
    // clang-format off
    LOG(INFO) << "Direct Odometry Node starting...";
    LOG(INFO) << "This node provides odometry for mapping and localization with GICP.";
    LOG(INFO) << "The node subscribe raw point cloud and imu.";
    LOG(INFO) << "Subscribe topics:";    
    LOG(INFO) << " - /cloud (sensor_msgs/PointCloud2) - realtime lidar scan subscription";    
    LOG(INFO) << " - /imu (sensor_msgs/Imu) - realtime IMU subscription";
    LOG(INFO) << "Publishing topics:";
    LOG(INFO) << " - /direct_odometry/pose (geometry_msgs/PoseWithCovarianceStamped).";

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Direct_Odometry_Node "), "Failed to start direct odometry node : %s", e.what());
    google::ShutdownGoogleLogging();
    return 1;
  }

  LOG(INFO) << "Direct Odometry Node shutting down";
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();
  return 0;
}