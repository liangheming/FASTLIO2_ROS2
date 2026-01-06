// ============================================================
// Registration Base Class - Interface for point cloud registration
// ============================================================

#pragma once

#include <Eigen/Dense>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace finder_localization {

/**
 * @brief 配准结果
 */
struct RegistrationResult {
  bool success = false;
  bool has_converged = false;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double fitness_score = 0.0;
  std::string message;

  // 对齐后的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud;
};

/**
 * @brief 配准基类 - 定义统一接口
 */
class RegistrationBase {
public:
  using Ptr = std::shared_ptr<RegistrationBase>;
  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

  virtual ~RegistrationBase() = default;

  /**
   * @brief 获取配准方法名称
   */
  virtual std::string getName() const = 0;

  /**
   * @brief 设置目标点云（地图）
   */
  virtual void setTarget(const PointCloud::Ptr& target) = 0;

  /**
   * @brief 设置源点云（当前扫描）
   */
  virtual void setSource(const PointCloud::Ptr& source) = 0;

  /**
   * @brief 执行配准
   * @param initial_guess 初始猜测
   * @return 配准结果
   */
  virtual RegistrationResult align(const Eigen::Matrix4f& initial_guess) = 0;

  /**
   * @brief 设置 fitness score 阈值
   */
  void setScoreThreshold(double threshold) { score_threshold_ = threshold; }

  /**
   * @brief 获取 score 阈值
   */
  double getScoreThreshold() const { return score_threshold_; }

protected:
  double score_threshold_ = 0.5;
};

}  // namespace finder_localization
