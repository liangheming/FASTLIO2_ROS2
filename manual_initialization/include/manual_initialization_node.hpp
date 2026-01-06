// ============================================================
// Manual Initialization Node - ROS2 Interface
//
// 功能：ROS2 节点，接收手动初始位姿并优化
//
// 输入：
//   - /initialpose: RViz 2D Pose Estimate
//   - /cloud: 当前点云
//   - /map: 全局地图
//
// 输出：
//   - /refined_initialpose: 优化后的初始位姿
//
// Author: AI Assistant
// Date: 2025-12-27
// ============================================================

#pragma once

#include <atomic>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "manual_initializer.hpp"

using namespace finder_localization;

class ManualInitializationNode : public rclcpp::Node {
public:
  ManualInitializationNode() : Node("manual_initialization_node") {
    RCLCPP_INFO(get_logger(), "Manual Initialization Node starting...");

    // 初始化 TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 加载参数
    loadParameters();

    // 创建算法实例
    initializer_ = std::make_unique<ManualInitializer>(config_);

    // 创建订阅
    initializeSubscribers();

    // 创建发布
    initializePublishers();

    RCLCPP_INFO(get_logger(), "Node initialized successfully");
    RCLCPP_INFO(get_logger(), "Waiting for map from globalmap_server_node...");
    printConfiguration();
  }

private:
  // ==================== 初始化方法 ====================

  void loadParameters() {
    declare_parameter("search_radius", 2.0);
    declare_parameter("yaw_search_range_deg", 90.0);
    declare_parameter("use_multiscale", true);
    declare_parameter("ndt_resolution", 1.5);
    declare_parameter("ndt_max_iterations", 20);
    declare_parameter("num_threads", 4);
    declare_parameter("global_frame_id", "map");
    declare_parameter("max_refinement_dist", 2.5);
    declare_parameter("max_refinement_yaw_deg", 90.0);
    declare_parameter("early_stop_score", 0.1);

    config_.search_radius = get_parameter("search_radius").as_double();
    config_.yaw_search_range = get_parameter("yaw_search_range_deg").as_double() * M_PI / 180.0;
    config_.use_multiscale = get_parameter("use_multiscale").as_bool();
    config_.ndt_resolution = get_parameter("ndt_resolution").as_double();
    config_.ndt_max_iterations = get_parameter("ndt_max_iterations").as_int();
    config_.num_threads = get_parameter("num_threads").as_int();
    config_.max_refinement_dist = get_parameter("max_refinement_dist").as_double();
    config_.max_refinement_yaw = get_parameter("max_refinement_yaw_deg").as_double() * M_PI / 180.0;
    config_.early_stop_score = get_parameter("early_stop_score").as_double();

    global_frame_id_ = get_parameter("global_frame_id").as_string();

    if (config_.max_refinement_dist < config_.search_radius) {
      RCLCPP_WARN(get_logger(), "max_refinement_dist (%.2f) < search_radius (%.2f)", config_.max_refinement_dist, config_.search_radius);
      config_.max_refinement_dist = config_.search_radius + 0.5;
    }

    if (config_.max_refinement_yaw < config_.yaw_search_range) {
      RCLCPP_WARN(get_logger(), "max_refinement_yaw (%.2f) < yaw_search_range (%.2f)", config_.max_refinement_yaw, config_.yaw_search_range);
      config_.max_refinement_yaw = config_.yaw_search_range + 0.5;
    }
  }

  void printConfiguration() {
    RCLCPP_INFO(get_logger(), "========== Configuration ==========");
    RCLCPP_INFO(get_logger(), "Search radius: %.2f m", config_.search_radius);
    RCLCPP_INFO(get_logger(), "Yaw search range: %.1f deg", config_.yaw_search_range * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "NDT resolution: %.2f m", config_.ndt_resolution);
    RCLCPP_INFO(get_logger(), "NDT max iterations: %d", config_.ndt_max_iterations);
    RCLCPP_INFO(get_logger(), "Num threads: %d", config_.num_threads);
    RCLCPP_INFO(get_logger(), "Global frame: %s", global_frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "===================================");
  }

  void initializeSubscribers() {
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(1), std::bind(&ManualInitializationNode::initialposeCallback, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS(),
                                                                     std::bind(&ManualInitializationNode::cloudCallback, this, std::placeholders::_1));

    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).transient_local(),
                                                                   std::bind(&ManualInitializationNode::mapCallback, this, std::placeholders::_1));
  }

  void initializePublishers() {
    refined_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("refined_initialpose", rclcpp::QoS(1).reliable());
    search_grid_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("manual_init/search_grid", rclcpp::QoS(1));

    rclcpp::QoS latched_qos(10);
    latched_qos.transient_local();

    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", latched_qos);
    source_cloud_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("manual_init/source_cloud", latched_qos);
    target_cloud_debug_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("manual_init/target_cloud", latched_qos);
  }

  // ==================== 回调方法 ====================

  void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (map_loaded_)
      return;

    RCLCPP_INFO(get_logger(), "Received map: %u points", msg->width * msg->height);

    map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *map_cloud_);

    RCLCPP_INFO(get_logger(), "Map loaded: %zu points", map_cloud_->size());
    map_loaded_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "========================================");
    RCLCPP_INFO(get_logger(), "Received initial pose from RViz");

    if (!map_loaded_) {
      RCLCPP_ERROR(get_logger(), "Map not loaded yet!");
      return;
    }

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(get_logger(), "Position: [%.2f, %.2f, %.2f]", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    RCLCPP_INFO(get_logger(), "Yaw: %.1f deg", yaw * 180.0 / M_PI);

    Eigen::Quaternionf quat(q.w(), q.x(), q.y(), q.z());
    last_initial_guess_ = Eigen::Matrix4f::Identity();
    last_initial_guess_.block<3, 3>(0, 0) = quat.toRotationMatrix();
    last_initial_guess_(0, 3) = msg->pose.pose.position.x;
    last_initial_guess_(1, 3) = msg->pose.pose.position.y;
    last_initial_guess_(2, 3) = msg->pose.pose.position.z;

    initial_pose_received_ = true;
  }

  // ==================== 核心处理方法（实现在 cpp 文件中）====================

  OptimizationResult performOptimization(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr extractLocalMap(const Eigen::Matrix4f &center);

  // ==================== 发布方法 ====================

  void publishAlignedCloud(const OptimizationResult &result) {
    if (!result.aligned_cloud || result.aligned_cloud->empty()) {
      RCLCPP_WARN(get_logger(), "No aligned cloud in result");
      return;
    }

    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(*result.aligned_cloud, aligned_msg);
    aligned_msg.header.frame_id = global_frame_id_;
    aligned_msg.header.stamp = now();

    aligned_cloud_pub_->publish(aligned_msg);
    RCLCPP_INFO(get_logger(), "Published aligned cloud: %zu points", result.aligned_cloud->size());
  }

  void publishDebugClouds(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud) {
    auto stamp = now();

    if (source_cloud && !source_cloud->empty()) {
      sensor_msgs::msg::PointCloud2 source_msg;
      pcl::toROSMsg(*source_cloud, source_msg);
      source_msg.header.frame_id = global_frame_id_;
      source_msg.header.stamp = stamp;
      source_cloud_debug_pub_->publish(source_msg);
    }

    if (target_cloud && !target_cloud->empty()) {
      sensor_msgs::msg::PointCloud2 target_msg;
      pcl::toROSMsg(*target_cloud, target_msg);
      target_msg.header.frame_id = global_frame_id_;
      target_msg.header.stamp = stamp;
      target_cloud_debug_pub_->publish(target_msg);
    }
  }

  void publishSearchGridVisualization(const Eigen::Matrix4f &center) {
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_id_;
    marker.header.stamp = now();
    marker.ns = "search_center";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center(0, 3);
    marker.pose.position.y = center(1, 3);
    marker.pose.position.z = center(2, 3);
    marker.scale.x = config_.search_radius * 2.0;
    marker.scale.y = config_.search_radius * 2.0;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.2;
    marker.lifetime = rclcpp::Duration::from_seconds(10.0);

    markers.markers.push_back(marker);
    search_grid_pub_->publish(markers);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped createPoseMessage(const OptimizationResult &result) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = global_frame_id_;
    msg.header.stamp = now();

    msg.pose.pose.position.x = result.transform(0, 3);
    msg.pose.pose.position.y = result.transform(1, 3);
    msg.pose.pose.position.z = result.transform(2, 3);

    Eigen::Matrix3d rot = result.transform.block<3, 3>(0, 0).cast<double>();
    Eigen::Quaterniond quat(rot);
    msg.pose.pose.orientation.x = quat.x();
    msg.pose.pose.orientation.y = quat.y();
    msg.pose.pose.orientation.z = quat.z();
    msg.pose.pose.orientation.w = quat.w();

    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    double score_factor = std::min(result.fitness_score / 0.3, 1.0);
    msg.pose.covariance[0] = 0.01 + score_factor * 0.1;
    msg.pose.covariance[7] = 0.01 + score_factor * 0.1;
    msg.pose.covariance[14] = 0.03 + score_factor * 0.2;
    msg.pose.covariance[35] = 0.001 + score_factor * 0.01;

    return msg;
  }

private:
  // 算法实例
  std::unique_ptr<ManualInitializer> initializer_;
  ManualInitializerConfig config_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr refined_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr search_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_cloud_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_cloud_debug_pub_;

  // 状态标志
  bool initial_pose_received_ = false;
  bool map_loaded_ = false;
  std::atomic<bool> is_processing_ = false;

  // 数据
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;
  Eigen::Matrix4f last_initial_guess_ = Eigen::Matrix4f::Identity();

  // 参数
  std::string global_frame_id_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
