// ============================================================
// GICP Registration - Generalized ICP point cloud registration
// ============================================================

#pragma once

#include "registration_base.hpp"

namespace finder_localization {

/**
 * @brief GICP 配准配置
 */
struct GicpConfig {
  int max_iterations = 50;
  double transform_epsilon = 1e-6;
  double euclidean_fitness_epsilon = 1e-6;
  int correspondence_randomness = 20;
  double max_correspondence_distance = 1.0;
};

/**
 * @brief GICP 配准实现
 */
class GicpRegistration : public RegistrationBase {
public:
  explicit GicpRegistration(const GicpConfig& config = GicpConfig());
  ~GicpRegistration() override = default;

  std::string getName() const override { return "GICP"; }

  void setTarget(const PointCloud::Ptr& target) override;
  void setSource(const PointCloud::Ptr& source) override;
  RegistrationResult align(const Eigen::Matrix4f& initial_guess) override;

  void updateConfig(const GicpConfig& config);

private:
  GicpConfig config_;
  PointCloud::Ptr source_;
  PointCloud::Ptr target_;
};

}  // namespace finder_localization
