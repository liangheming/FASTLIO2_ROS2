/***********************************************************
 * Ground Constraint Module for DLIO
 * Purpose: Reduce z-axis drift using ground plane constraint
 *
 * Features:
 * - Fast ground segmentation for multi-beam LiDAR
 * - Plane fitting and height constraint
 * - Real-time performance optimized
 ***********************************************************/

#ifndef GROUND_CONSTRAINT_H
#define GROUND_CONSTRAINT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace dlio
{

class GroundConstraint
{
public:
  struct Config
  {
    // 地面分割参数
    float ground_z_min = -2.0;  // 地面点最低高度（相对传感器）
    float ground_z_max = -0.3;  // 地面点最高高度
    int ground_ring_min = 0;    // 地面点所在的最小ring（如果有ring信息）
    int ground_ring_max = 5;    // 地面点所在的最大ring（多线雷达最下面几线）

    // 平面拟合参数
    int min_ground_points = 50;       // 最小地面点数量
    float plane_fit_threshold = 0.1;  // 平面拟合inlier阈值（米）

    // z轴约束参数
    float sensor_height = 1.5;        // 传感器距地面高度（米）
    float z_drift_threshold = 0.2;    // 触发修正的z漂移阈值（米）
    float z_correction_weight = 0.3;  // z修正权重[0-1]，越大修正越强

    // 实时性优化
    bool use_ring_filter = true;    // 优先使用ring信息（更快）
    bool enable_constraint = true;  // 总开关
  };

  GroundConstraint(const Config& config);

  // 主接口：提取地面点
  void segmentGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& non_ground_cloud);

  // 拟合地面平面
  bool fitGroundPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud, Eigen::Vector4f& plane_coefficients);

  // 应用z轴约束修正
  void applyZConstraint(Eigen::Vector3f& position, const Eigen::Vector4f& ground_plane);

  // 获取当前地面高度
  float getCurrentGroundHeight(const Eigen::Vector3f& position, const Eigen::Vector4f& ground_plane);

  // 更新配置
  void updateConfig(const Config& config)
  {
    config_ = config;
  }

  // 获取统计信息
  struct Stats
  {
    int ground_points_count = 0;
    float plane_fit_error = 0.0;
    float z_correction = 0.0;
    bool constraint_applied = false;
  };
  const Stats& getStats() const
  {
    return stats_;
  }

private:
  Config config_;
  Stats stats_;

  // 简单快速的平面拟合（PCA或最小二乘）
  bool fastPlaneFit(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Vector4f& plane);
};

}  // namespace dlio

#endif  // GROUND_CONSTRAINT_H