/***********************************************************
 * Ground Constraint Module Implementation
 ***********************************************************/

#include "dlio/ground_constraint.h"
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <algorithm>
#include <numeric>

namespace dlio
{

GroundConstraint::GroundConstraint(const Config& config) : config_(config)
{
}

void GroundConstraint::segmentGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_in,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
                                     pcl::PointCloud<pcl::PointXYZI>::Ptr& non_ground_cloud)
{
  if (!config_.enable_constraint)
  {
    *non_ground_cloud = *cloud_in;
    return;
  }

  ground_cloud->clear();
  non_ground_cloud->clear();
  ground_cloud->reserve(cloud_in->size() / 5);
  non_ground_cloud->reserve(cloud_in->size());

  // 方法1: 如果有ring信息（Velodyne/Ouster/Hesai格式）
  // 注意：pcl::PointXYZI 不包含 ring 信息，这里默认使用 z 高度方法
  // 如果需要使用 ring 信息，需要使用自定义点云类型（如 PointXYZIRT）
  bool has_ring = false;

  if (config_.use_ring_filter && has_ring)
  {
    // 使用ring信息快速提取（推荐，最快）
    // 假设点云类型包含ring信息，需要用PointXYZIRT或自定义类型
    // 这里用z高度作为backup方案
    for (const auto& point : cloud_in->points)
    {
      if (point.z >= config_.ground_z_min && point.z <= config_.ground_z_max)
      {
        ground_cloud->push_back(point);
      }
      else
      {
        non_ground_cloud->push_back(point);
      }
    }
  }
  else
  {
    // 方法2: 简单z高度滤波（快速有效，适合平坦地面）
    for (const auto& point : cloud_in->points)
    {
      if (point.z >= config_.ground_z_min && point.z <= config_.ground_z_max)
      {
        ground_cloud->push_back(point);
      }
      else
      {
        non_ground_cloud->push_back(point);
      }
    }
  }

  stats_.ground_points_count = ground_cloud->size();
}

bool GroundConstraint::fitGroundPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr& ground_cloud,
                                      Eigen::Vector4f& plane_coefficients)
{
  if (ground_cloud->size() < config_.min_ground_points)
  {
    return false;
  }

  // 使用快速平面拟合（最小二乘法）
  return fastPlaneFit(ground_cloud, plane_coefficients);
}

bool GroundConstraint::fastPlaneFit(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, Eigen::Vector4f& plane)
{
  // 计算质心
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  // 构建协方差矩阵
  Eigen::Matrix3f covariance;
  covariance.setZero();

  for (const auto& point : cloud->points)
  {
    Eigen::Vector3f pt(point.x - centroid[0], point.y - centroid[1], point.z - centroid[2]);
    covariance += pt * pt.transpose();
  }
  covariance /= cloud->size();

  // 特征值分解，最小特征值对应的特征向量就是法向量
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
  Eigen::Vector3f normal = eigen_solver.eigenvectors().col(0);

  // 确保法向量朝上（z分量为正）
  if (normal[2] < 0)
  {
    normal = -normal;
  }

  // 平面方程: ax + by + cz + d = 0
  plane[0] = normal[0];
  plane[1] = normal[1];
  plane[2] = normal[2];
  plane[3] = -(normal[0] * centroid[0] + normal[1] * centroid[1] + normal[2] * centroid[2]);

  // 计算拟合误差（所有点到平面的平均距离）
  float error_sum = 0.0;
  for (const auto& point : cloud->points)
  {
    float dist = std::abs(plane[0] * point.x + plane[1] * point.y + plane[2] * point.z + plane[3]);
    error_sum += dist;
  }
  stats_.plane_fit_error = error_sum / cloud->size();

  // 如果拟合误差太大，认为失败
  if (stats_.plane_fit_error > config_.plane_fit_threshold)
  {
    return false;
  }

  return true;
}

float GroundConstraint::getCurrentGroundHeight(const Eigen::Vector3f& position, const Eigen::Vector4f& ground_plane)
{
  // 点到平面的有符号距离
  float distance = (ground_plane[0] * position[0] + ground_plane[1] * position[1] + ground_plane[2] * position[2] +
                    ground_plane[3]) /
                   std::sqrt(ground_plane[0] * ground_plane[0] + ground_plane[1] * ground_plane[1] +
                             ground_plane[2] * ground_plane[2]);

  return distance;
}

void GroundConstraint::applyZConstraint(Eigen::Vector3f& position, const Eigen::Vector4f& ground_plane)
{
  if (!config_.enable_constraint)
  {
    stats_.constraint_applied = false;
    return;
  }

  // 计算当前高度
  float current_height = getCurrentGroundHeight(position, ground_plane);

  // 计算漂移量
  float height_drift = current_height - config_.sensor_height;

  // 判断是否需要修正
  if (std::abs(height_drift) > config_.z_drift_threshold)
  {
    // 软修正：按权重拉回
    float correction = -height_drift * config_.z_correction_weight;
    position[2] += correction;

    stats_.z_correction = correction;
    stats_.constraint_applied = true;
  }
  else
  {
    stats_.z_correction = 0.0;
    stats_.constraint_applied = false;
  }
}

}  // namespace dlio