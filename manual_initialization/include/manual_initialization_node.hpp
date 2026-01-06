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

#include "registration/registration_base.hpp"
#include "registration/ndt_registration.hpp"
#include "registration/gicp_registration.hpp"
#include "registration/small_gicp_registration.hpp"
#include "registration/cascade_registration.hpp"

using namespace finder_localization;

class ManualInitializationNode : public rclcpp::Node {
public:
  ManualInitializationNode() : Node("manual_initialization_node") {
    RCLCPP_INFO(get_logger(), "Manual Initialization Node starting...");

    // 初始化 TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 加载参数并创建配准实例
    loadParameters();

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
    // 配准方法选择: "ndt", "gicp", "small_gicp", "vgicp", "cascade"
    declare_parameter("registration_method", "cascade");
    declare_parameter("score_threshold", 2.5);
    declare_parameter("num_threads", 4);

    // NDT 参数
    declare_parameter("ndt_resolution", 1.0);
    declare_parameter("ndt_max_iterations", 30);

    // GICP 参数 (PCL)
    declare_parameter("gicp_max_iterations", 50);
    declare_parameter("gicp_max_correspondence_distance", 1.0);

    // Small GICP / VGICP 参数
    declare_parameter("small_gicp_max_iterations", 30);
    declare_parameter("small_gicp_max_correspondence_distance", 1.0);
    declare_parameter("small_gicp_voxel_resolution", 1.0);
    declare_parameter("small_gicp_num_neighbors", 20);

    // Cascade 参数
    declare_parameter("cascade_coarse_method", "ndt");      // "ndt" or "vgicp"
    declare_parameter("cascade_fine_method", "small_gicp"); // "gicp" or "small_gicp"
    declare_parameter("cascade_coarse_max_iterations", 30);
    declare_parameter("cascade_fine_max_iterations", 50);
    declare_parameter("cascade_coarse_score_threshold", 3.0);

    // 点云滤波参数
    declare_parameter("voxel_leaf_size", 0.15);
    declare_parameter("scan_max_range", 40.0);
    declare_parameter("scan_min_range", 1.0);
    declare_parameter("min_scan_points", 500);

    // 地图处理参数
    declare_parameter("map_voxel_size", 0.5);
    declare_parameter("local_map_radius", 30.0);

    // 坐标系
    declare_parameter("global_frame_id", "map");

    // 读取参数
    registration_method_ = get_parameter("registration_method").as_string();
    score_threshold_ = get_parameter("score_threshold").as_double();
    int num_threads = get_parameter("num_threads").as_int();

    voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
    scan_max_range_ = get_parameter("scan_max_range").as_double();
    scan_min_range_ = get_parameter("scan_min_range").as_double();
    min_scan_points_ = get_parameter("min_scan_points").as_int();
    map_voxel_size_ = get_parameter("map_voxel_size").as_double();
    local_map_radius_ = get_parameter("local_map_radius").as_double();
    global_frame_id_ = get_parameter("global_frame_id").as_string();

    // 根据方法创建配准实例
    if (registration_method_ == "cascade" || registration_method_ == "CASCADE") {
      CascadeConfig config;
      config.coarse_method = get_parameter("cascade_coarse_method").as_string();
      config.fine_method = get_parameter("cascade_fine_method").as_string();
      config.coarse_max_iterations = get_parameter("cascade_coarse_max_iterations").as_int();
      config.fine_max_iterations = get_parameter("cascade_fine_max_iterations").as_int();
      config.coarse_score_threshold = get_parameter("cascade_coarse_score_threshold").as_double();
      config.num_threads = num_threads;
      config.max_correspondence_distance = get_parameter("small_gicp_max_correspondence_distance").as_double();
      config.ndt_resolution = get_parameter("ndt_resolution").as_double();
      config.vgicp_voxel_resolution = get_parameter("small_gicp_voxel_resolution").as_double();
      config.num_neighbors = get_parameter("small_gicp_num_neighbors").as_int();
      registration_ = std::make_shared<CascadeRegistration>(config);
    } else if (registration_method_ == "small_gicp" || registration_method_ == "SMALL_GICP") {
      SmallGicpConfig config;
      config.type = "GICP";
      config.max_iterations = get_parameter("small_gicp_max_iterations").as_int();
      config.num_threads = num_threads;
      config.num_neighbors = get_parameter("small_gicp_num_neighbors").as_int();
      config.max_correspondence_distance = get_parameter("small_gicp_max_correspondence_distance").as_double();
      registration_ = std::make_shared<SmallGicpRegistration>(config);
    } else if (registration_method_ == "vgicp" || registration_method_ == "VGICP") {
      SmallGicpConfig config;
      config.type = "VGICP";
      config.max_iterations = get_parameter("small_gicp_max_iterations").as_int();
      config.num_threads = num_threads;
      config.num_neighbors = get_parameter("small_gicp_num_neighbors").as_int();
      config.max_correspondence_distance = get_parameter("small_gicp_max_correspondence_distance").as_double();
      config.voxel_resolution = get_parameter("small_gicp_voxel_resolution").as_double();
      registration_ = std::make_shared<SmallGicpRegistration>(config);
    } else if (registration_method_ == "gicp" || registration_method_ == "GICP") {
      GicpConfig config;
      config.max_iterations = get_parameter("gicp_max_iterations").as_int();
      config.max_correspondence_distance = get_parameter("gicp_max_correspondence_distance").as_double();
      registration_ = std::make_shared<GicpRegistration>(config);
    } else {
      // Default to NDT
      NdtConfig config;
      config.resolution = get_parameter("ndt_resolution").as_double();
      config.max_iterations = get_parameter("ndt_max_iterations").as_int();
      config.num_threads = num_threads;
      registration_ = std::make_shared<NdtRegistration>(config);
    }

    registration_->setScoreThreshold(score_threshold_);
  }

  void printConfiguration() {
    RCLCPP_INFO(get_logger(), "========== Configuration ==========");
    RCLCPP_INFO(get_logger(), "Registration method: %s", registration_->getName().c_str());
    RCLCPP_INFO(get_logger(), "Score threshold: %.3f", score_threshold_);
    RCLCPP_INFO(get_logger(), "Global frame: %s", global_frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "--- Point Cloud Filtering ---");
    RCLCPP_INFO(get_logger(), "Voxel leaf size: %.2f m", voxel_leaf_size_);
    RCLCPP_INFO(get_logger(), "Scan range: [%.1f, %.1f] m", scan_min_range_, scan_max_range_);
    RCLCPP_INFO(get_logger(), "Min scan points: %d", min_scan_points_);
    RCLCPP_INFO(get_logger(), "--- Map Processing ---");
    RCLCPP_INFO(get_logger(), "Map voxel size: %.2f m", map_voxel_size_);
    RCLCPP_INFO(get_logger(), "Local map radius: %.1f m", local_map_radius_);
    RCLCPP_INFO(get_logger(), "===================================");
  }

  void initializeSubscribers() {
    initialpose_sub_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(1), std::bind(&ManualInitializationNode::initialposeCallback, this, std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS(), std::bind(&ManualInitializationNode::cloudCallback, this, std::placeholders::_1));

    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("map", rclcpp::QoS(1).transient_local(), std::bind(&ManualInitializationNode::mapCallback, this, std::placeholders::_1));
  }

  void initializePublishers() {
    refined_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("refined_initialpose", rclcpp::QoS(1).reliable());

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

  /**
   * @brief 将点云从 lidar frame 转换到 base_link frame 并滤波
   *
   * @param input 输入点云消息（lidar frame）
   * @param output 输出点云（base_link frame）
   * @return true 转换成功
   */
  bool transformAndFilterPC(const sensor_msgs::msg::PointCloud2::SharedPtr &input, pcl::PointCloud<pcl::PointXYZI>::Ptr &output);

  RegistrationResult performOptimization(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr extractLocalMap(const Eigen::Matrix4f &center);

  // ==================== 发布方法 ====================

  void publishAlignedCloud(const RegistrationResult &result) {
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

  geometry_msgs::msg::PoseWithCovarianceStamped createPoseMessage(const RegistrationResult &result) {
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
  // 配准算法实例
  RegistrationBase::Ptr registration_;

  // ROS 接口
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr refined_pose_pub_;
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
  std::string registration_method_;
  double score_threshold_ = 2.5;

  // 点云滤波参数
  double voxel_leaf_size_ = 0.15;
  double scan_max_range_ = 40.0;
  double scan_min_range_ = 1.0;
  int min_scan_points_ = 500;

  // 地图处理参数
  double map_voxel_size_ = 0.5;
  double local_map_radius_ = 30.0;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
