// ============================================================
// Manual Initialization Node - Implementation
// ============================================================

#include "manual_initialization_node.hpp"

bool ManualInitializationNode::transformAndFilterPC(const sensor_msgs::msg::PointCloud2::SharedPtr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr &output) {
  try {
    // 1. 转换为 PCL 格式
    pcl::fromROSMsg(*input, *output);

    size_t original_size = output->size();
    std::string source_frame = input->header.frame_id;
    std::string target_frame = "base_link";

    // 2. 坐标变换：lidar frame -> base_link
    if (source_frame != target_frame) {
      if (!tf_buffer_->canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
        RCLCPP_ERROR(get_logger(), "TF not available: %s -> %s", source_frame.c_str(), target_frame.c_str());
        return false;
      }

      auto transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

      Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform.transform);
      Eigen::Matrix4f transformation = transform_eigen.matrix().cast<float>();

      pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
      pcl::transformPointCloud(*output, transformed_cloud, transformation);
      *output = std::move(transformed_cloud);

      RCLCPP_INFO(get_logger(), "Transformed cloud from %s to %s", source_frame.c_str(), target_frame.c_str());
    }

    // 3. 距离滤波：去除无效点和超出范围的点
    auto range_filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    range_filtered->reserve(output->size());

    for (const auto &point : output->points) {
      // 检查点是否有效
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      // 计算水平距离（只考虑 x, y）
      double range = std::sqrt(point.x * point.x + point.y * point.y);

      // 距离范围检查
      if (range >= scan_min_range_ && range <= scan_max_range_) {
        range_filtered->push_back(point);
      }
    }

    size_t after_range_filter = range_filtered->size();

    if (range_filtered->empty()) {
      RCLCPP_ERROR(get_logger(), "No points left after range filtering");
      return false;
    }

    // 4. 体素降采样
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.setInputCloud(range_filtered);
    voxel.filter(*output);

    RCLCPP_INFO(get_logger(), "Point cloud filtering: %zu -> %zu (range) -> %zu (voxel)", original_size, after_range_filter, output->size());

    return static_cast<int>(output->size()) >= min_scan_points_;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "transformAndFilterPC failed: %s", e.what());
    return false;
  }
}

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

  // 1. 转换点云到 base_link 坐标系并滤波
  auto source_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (!transformAndFilterPC(cloud_msg, source_cloud)) {
    result.message = "Failed to transform/filter point cloud";
    return result;
  }

  RCLCPP_INFO(get_logger(), "Source cloud (base_link frame): %zu points", source_cloud->size());

  // 2. 提取局部地图（map frame）
  auto target_cloud = extractLocalMap(last_initial_guess_);
  if (!target_cloud || target_cloud->size() < 1000) {
    result.message = "Local map too small";
    return result;
  }

  RCLCPP_INFO(get_logger(), "Target cloud (map frame): %zu points", target_cloud->size());
  RCLCPP_INFO(get_logger(), "Initial guess T_map_baselink: [%.2f, %.2f, %.2f]", last_initial_guess_(0, 3), last_initial_guess_(1, 3), last_initial_guess_(2, 3));

  // 3. 发布调试点云（将 source 用 initial_guess 变换到 map frame 用于可视化）
  // auto source_in_map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  // pcl::transformPointCloud(*source_cloud, *source_in_map, last_initial_guess_);
  // publishDebugClouds(source_in_map, target_cloud);

  // 4. 可视化搜索区域
  publishSearchGridVisualization(last_initial_guess_);

  // 5. 调用简单优化
  // source_cloud 在 base_link frame，target_cloud 在 map frame
  // initial_guess 是 T_map_baselink
  // NDT 配准：找到最优的 T_map_baselink 使得 source_cloud 对齐到 target_cloud
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

  // 1. 距离提取
  for (const auto &pt : *map_cloud_) {
    double dx = pt.x - cx;
    double dy = pt.y - cy;
    double dz = pt.z - cz;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < radius) {
      local_map->push_back(pt);
    }
  }

  size_t before_downsample = local_map->size();

  // 2. 体素降采样
  if (map_voxel_size_ > 0.0 && !local_map->empty()) {
    auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
    voxel.setInputCloud(local_map);
    voxel.filter(*downsampled);

    RCLCPP_INFO(get_logger(), "Local map: %zu -> %zu points (voxel: %.2f m)", before_downsample, downsampled->size(), map_voxel_size_);

    return downsampled;
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
