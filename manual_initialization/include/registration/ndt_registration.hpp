// ============================================================
// NDT Registration - NDT-based point cloud registration
// ============================================================

#pragma once

#include "registration_base.hpp"

namespace finder_localization {

/**
 * @brief NDT 配准配置
 */
struct NdtConfig {
  double resolution = 1.0;
  double step_size = 0.1;
  double transform_epsilon = 0.01;
  int max_iterations = 30;
  int num_threads = 4;
};

/**
 * @brief NDT 配准实现
 */
class NdtRegistration : public RegistrationBase {
public:
  explicit NdtRegistration(const NdtConfig& config = NdtConfig());
  ~NdtRegistration() override = default;

  std::string getName() const override { return "NDT"; }

  void setTarget(const PointCloud::Ptr& target) override;
  void setSource(const PointCloud::Ptr& source) override;
  RegistrationResult align(const Eigen::Matrix4f& initial_guess) override;

  void updateConfig(const NdtConfig& config);

private:
  NdtConfig config_;
  PointCloud::Ptr source_;
  PointCloud::Ptr target_;
};

}  // namespace finder_localization
