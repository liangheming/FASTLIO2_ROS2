// ============================================================
// Small GICP Registration - Implementation
// ============================================================

#include "registration/small_gicp_registration.hpp"
#include <iostream>
#include <pcl/common/transforms.h>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace finder_localization {

SmallGicpRegistration::SmallGicpRegistration(const SmallGicpConfig &config) : config_(config) {}

void SmallGicpRegistration::updateConfig(const SmallGicpConfig &config) { config_ = config; }

std::string SmallGicpRegistration::getName() const { return "Small" + config_.type; }

void SmallGicpRegistration::setTarget(const PointCloud::Ptr &target) { target_ = target; }

void SmallGicpRegistration::setSource(const PointCloud::Ptr &source) { source_ = source; }

RegistrationResult SmallGicpRegistration::align(const Eigen::Matrix4f &initial_guess) {
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

  std::cout << "[" << getName() << "] Aligning..." << std::endl;
  std::cout << "  Source: " << source_->size() << " points" << std::endl;
  std::cout << "  Target: " << target_->size() << " points" << std::endl;

  // 配置 small_gicp
  small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI> reg;
  reg.setRegistrationType(config_.type);
  reg.setMaximumIterations(config_.max_iterations);
  reg.setNumThreads(config_.num_threads);
  reg.setCorrespondenceRandomness(config_.num_neighbors);
  reg.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
  reg.setTransformationEpsilon(config_.transformation_epsilon);
  reg.setRotationEpsilon(config_.rotation_epsilon * M_PI / 180.0); // Convert to radians
  reg.setVerbosity(config_.verbose);

  if (config_.type == "VGICP") {
    reg.setVoxelResolution(config_.voxel_resolution);
  }

  reg.setInputSource(source_);
  reg.setInputTarget(target_);

  // 执行配准
  pcl::PointCloud<pcl::PointXYZI> aligned;
  reg.align(aligned, initial_guess);

  result.has_converged = reg.hasConverged();
  result.fitness_score = reg.getFitnessScore();

  if (result.has_converged) {
    result.transform = reg.getFinalTransformation();

    // 生成对齐后的点云
    result.aligned_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*source_, *result.aligned_cloud, result.transform);

    std::cout << "[" << getName() << "] Converged: YES" << std::endl;
    std::cout << "  Fitness score: " << result.fitness_score << " success threshold: " << score_threshold_ << std::endl;
    std::cout << "  Iterations: " << reg.getRegistrationResult().iterations << std::endl;

    if (result.fitness_score < score_threshold_) {
      result.success = true;
      result.message = getName() + " registration successful";
    } else {
      result.message = "Fitness score too high: " + std::to_string(result.fitness_score);
    }
  } else {
    result.message = getName() + " failed to converge";
    std::cout << "[" << getName() << "] Converged: NO" << std::endl;
  }

  return result;
}

} // namespace finder_localization
