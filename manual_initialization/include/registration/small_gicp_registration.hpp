// ============================================================
// Small GICP Registration - Fast GICP/VGICP implementation
// Using small_gicp library by Kenji Koide
// ============================================================

#pragma once

#include "registration_base.hpp"

namespace finder_localization {

/**
 * @brief Small GICP 配准配置
 */
struct SmallGicpConfig {
  std::string type = "VGICP";           // "GICP" or "VGICP"
  int max_iterations = 30;
  int num_threads = 4;
  int num_neighbors = 20;               // For covariance estimation
  double max_correspondence_distance = 1.0;
  double voxel_resolution = 1.0;        // For VGICP only
  double rotation_epsilon = 0.1;        // degrees
  double transformation_epsilon = 1e-4;
  bool verbose = false;
};

/**
 * @brief Small GICP 配准实现
 *
 * 支持两种模式：
 * - GICP: 标准 Generalized ICP，精度高
 * - VGICP: Voxelized GICP，速度更快，适合大点云
 */
class SmallGicpRegistration : public RegistrationBase {
public:
  explicit SmallGicpRegistration(const SmallGicpConfig& config = SmallGicpConfig());
  ~SmallGicpRegistration() override = default;

  std::string getName() const override;

  void setTarget(const PointCloud::Ptr& target) override;
  void setSource(const PointCloud::Ptr& source) override;
  RegistrationResult align(const Eigen::Matrix4f& initial_guess) override;

  void updateConfig(const SmallGicpConfig& config);

private:
  SmallGicpConfig config_;
  PointCloud::Ptr source_;
  PointCloud::Ptr target_;
};

}  // namespace finder_localization
