// ============================================================
// Manual Initialization Node - Implementation
// ============================================================

#include "manual_initialization_node.hpp"

void ManualInitializationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!initial_pose_received_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for initial pose...");
    return;
  }

  if (is_processing_)
    return;

  is_processing_ = true;

  std::thread([this, cloud = msg]() {
    auto result = performOptimization(cloud);

    if (result.success) {
      auto refined_msg = createPoseMessage(result);
      refined_pose_pub_->publish(refined_msg);
      publishAlignedCloud(result);
      RCLCPP_INFO(get_logger(), "Refined pose published");
    } else {
      RCLCPP_ERROR(get_logger(), "Optimization failed: %s", result.message.c_str());
    }

    is_processing_ = false;
    initial_pose_received_ = false;
  }).detach();
}

OptimizationResult ManualInitializationNode::performOptimization(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg) {
  OptimizationResult result;

  // 1. 转换点云
  auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*cloud_msg, *source_cloud);

  if (!source_cloud || source_cloud->size() < 500) {
    result.message = "Insufficient scan points";
    return result;
  }

  RCLCPP_INFO(get_logger(), "Source cloud: %zu points", source_cloud->size());

  // 2. 降采样
  if (source_cloud->size() > 15000) {
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel.setInputCloud(source_cloud);
    voxel.filter(*filtered);
    source_cloud = filtered;
    RCLCPP_INFO(get_logger(), "Downsampled to: %zu points", source_cloud->size());
  }

  // 3. 提取局部地图
  auto target_cloud = extractLocalMap(last_initial_guess_);
  if (!target_cloud || target_cloud->size() < 1000) {
    result.message = "Local map too small";
    return result;
  }

  RCLCPP_INFO(get_logger(), "Target cloud: %zu points", target_cloud->size());
  RCLCPP_INFO(get_logger(), "Initial guess: [%.2f, %.2f, %.2f]", last_initial_guess_(0, 3), last_initial_guess_(1, 3), last_initial_guess_(2, 3));

  // 4. 发布调试点云
  auto source_in_map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::transformPointCloud(*source_cloud, *source_in_map, last_initial_guess_);
  publishDebugClouds(source_in_map, target_cloud);

  // 5. 可视化搜索区域
  publishSearchGridVisualization(last_initial_guess_);

  // 6. 调用简单优化
  return initializer_->simpleOptimize(source_cloud, target_cloud, last_initial_guess_);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ManualInitializationNode::extractLocalMap(const Eigen::Matrix4f &center) {
  if (!map_loaded_)
    return nullptr;

  auto local_map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  double cx = center(0, 3);
  double cy = center(1, 3);
  double cz = center(2, 3);
  double radius = config_.search_radius + 30.0;

  for (const auto &pt : *map_cloud_) {
    double dx = pt.x - cx;
    double dy = pt.y - cy;
    double dz = pt.z - cz;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < radius) {
      local_map->push_back(pt);
    }
  }

  return local_map;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualInitializationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
