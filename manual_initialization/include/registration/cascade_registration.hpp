// ============================================================
// Cascade Registration - Two-stage coarse-to-fine registration
//
// 两阶段级联配准：
//   1. 粗配准（NDT/VGICP）：收敛域大，鲁棒性好
//   2. 精配准（GICP）：精度高
// ============================================================

#pragma once

#include "registration_base.hpp"

namespace finder_localization {

/**
 * @brief 级联配准配置
 */
struct CascadeConfig {
  // 粗配准阶段
  std::string coarse_method = "ndt";     // "ndt" or "vgicp"
  int coarse_max_iterations = 30;
  double coarse_score_threshold = 3.0;   // 粗配准允许较高的 score

  // 精配准阶段
  std::string fine_method = "small_gicp"; // "gicp" or "small_gicp"
  int fine_max_iterations = 50;

  // 通用参数
  int num_threads = 4;
  double max_correspondence_distance = 1.0;
  double ndt_resolution = 1.0;
  double vgicp_voxel_resolution = 1.0;
  int num_neighbors = 20;
};

/**
 * @brief 级联配准实现
 *
 * 两阶段配准策略，结合粗配准的鲁棒性和精配准的精度
 */
class CascadeRegistration : public RegistrationBase {
public:
  explicit CascadeRegistration(const CascadeConfig& config = CascadeConfig());
  ~CascadeRegistration() override = default;

  std::string getName() const override;

  void setTarget(const PointCloud::Ptr& target) override;
  void setSource(const PointCloud::Ptr& source) override;
  RegistrationResult align(const Eigen::Matrix4f& initial_guess) override;

  void updateConfig(const CascadeConfig& config);

private:
  RegistrationBase::Ptr createCoarseRegistration();
  RegistrationBase::Ptr createFineRegistration();

  CascadeConfig config_;
  PointCloud::Ptr source_;
  PointCloud::Ptr target_;

  RegistrationBase::Ptr coarse_registration_;
  RegistrationBase::Ptr fine_registration_;
};

}  // namespace finder_localization
