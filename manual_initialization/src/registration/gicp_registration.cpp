// ============================================================
// GICP Registration - Implementation
// ============================================================

#include "registration/gicp_registration.hpp"
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>

namespace finder_localization {

GicpRegistration::GicpRegistration(const GicpConfig& config) : config_(config) {}

void GicpRegistration::updateConfig(const GicpConfig& config) {
  config_ = config;
}

void GicpRegistration::setTarget(const PointCloud::Ptr& target) {
  target_ = target;
}

void GicpRegistration::setSource(const PointCloud::Ptr& source) {
  source_ = source;
}

RegistrationResult GicpRegistration::align(const Eigen::Matrix4f& initial_guess) {
  RegistrationResult result;
  result.transform = initial_guess;

  if (!source_ || source_->empty()) {
    result.message = "Source cloud is empty";
    return result;
  }

  if (!target_ || target_->empty()) {
    result.message = "Target cloud is empty";
    return result;
  }

  std::cout << "[GICP] Aligning..." << std::endl;
  std::cout << "  Source: " << source_->size() << " points" << std::endl;
  std::cout << "  Target: " << target_->size() << " points" << std::endl;

  // 配置 GICP
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
  gicp.setMaximumIterations(config_.max_iterations);
  gicp.setTransformationEpsilon(config_.transform_epsilon);
  gicp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
  gicp.setCorrespondenceRandomness(config_.correspondence_randomness);
  gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);

  gicp.setInputSource(source_);
  gicp.setInputTarget(target_);

  // 执行配准
  pcl::PointCloud<pcl::PointXYZI> aligned;
  gicp.align(aligned, initial_guess);

  result.has_converged = gicp.hasConverged();
  result.fitness_score = gicp.getFitnessScore();

  if (result.has_converged) {
    result.transform = gicp.getFinalTransformation();

    // 生成对齐后的点云
    result.aligned_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*source_, *result.aligned_cloud, result.transform);

    std::cout << "[GICP] Converged: YES" << std::endl;
    std::cout << "  Fitness score: " << result.fitness_score << std::endl;

    if (result.fitness_score < score_threshold_) {
      result.success = true;
      result.message = "GICP registration successful";
    } else {
      result.message = "Fitness score too high: " + std::to_string(result.fitness_score);
    }
  } else {
    result.message = "GICP failed to converge";
    std::cout << "[GICP] Converged: NO" << std::endl;
  }

  return result;
}

}  // namespace finder_localization
