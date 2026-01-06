// ============================================================
// NDT Registration - Implementation
// ============================================================

#include "registration/ndt_registration.hpp"
#include <iostream>
#include <pcl/common/transforms.h>
#include <pclomp/ndt_omp.h>

namespace finder_localization {

NdtRegistration::NdtRegistration(const NdtConfig& config) : config_(config) {}

void NdtRegistration::updateConfig(const NdtConfig& config) {
  config_ = config;
}

void NdtRegistration::setTarget(const PointCloud::Ptr& target) {
  target_ = target;
}

void NdtRegistration::setSource(const PointCloud::Ptr& source) {
  source_ = source;
}

RegistrationResult NdtRegistration::align(const Eigen::Matrix4f& initial_guess) {
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

  std::cout << "[NDT] Aligning..." << std::endl;
  std::cout << "  Source: " << source_->size() << " points" << std::endl;
  std::cout << "  Target: " << target_->size() << " points" << std::endl;

  // 配置 NDT
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setResolution(config_.resolution);
  ndt.setStepSize(config_.step_size);
  ndt.setTransformationEpsilon(config_.transform_epsilon);
  ndt.setMaximumIterations(config_.max_iterations);
  ndt.setNumThreads(config_.num_threads);

  ndt.setInputSource(source_);
  ndt.setInputTarget(target_);

  // 执行配准
  pcl::PointCloud<pcl::PointXYZI> aligned;
  ndt.align(aligned, initial_guess);

  result.has_converged = ndt.hasConverged();
  result.fitness_score = ndt.getFitnessScore();

  if (result.has_converged) {
    result.transform = ndt.getFinalTransformation();

    // 生成对齐后的点云
    result.aligned_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*source_, *result.aligned_cloud, result.transform);

    std::cout << "[NDT] Converged: YES" << std::endl;
    std::cout << "  Fitness score: " << result.fitness_score << std::endl;

    if (result.fitness_score < score_threshold_) {
      result.success = true;
      result.message = "NDT registration successful";
    } else {
      result.message = "Fitness score too high: " + std::to_string(result.fitness_score);
    }
  } else {
    result.message = "NDT failed to converge";
    std::cout << "[NDT] Converged: NO" << std::endl;
  }

  return result;
}

}  // namespace finder_localization
