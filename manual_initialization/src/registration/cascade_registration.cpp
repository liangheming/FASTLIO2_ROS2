// ============================================================
// Cascade Registration - Implementation
// ============================================================

#include "registration/cascade_registration.hpp"
#include "registration/ndt_registration.hpp"
#include "registration/gicp_registration.hpp"
#include "registration/small_gicp_registration.hpp"
#include <iostream>

namespace finder_localization {

CascadeRegistration::CascadeRegistration(const CascadeConfig& config) : config_(config) {
  coarse_registration_ = createCoarseRegistration();
  fine_registration_ = createFineRegistration();
}

void CascadeRegistration::updateConfig(const CascadeConfig& config) {
  config_ = config;
  coarse_registration_ = createCoarseRegistration();
  fine_registration_ = createFineRegistration();
}

std::string CascadeRegistration::getName() const {
  return "Cascade(" + coarse_registration_->getName() + "+" + fine_registration_->getName() + ")";
}

RegistrationBase::Ptr CascadeRegistration::createCoarseRegistration() {
  if (config_.coarse_method == "vgicp" || config_.coarse_method == "VGICP") {
    SmallGicpConfig cfg;
    cfg.type = "VGICP";
    cfg.max_iterations = config_.coarse_max_iterations;
    cfg.num_threads = config_.num_threads;
    cfg.num_neighbors = config_.num_neighbors;
    cfg.max_correspondence_distance = config_.max_correspondence_distance;
    cfg.voxel_resolution = config_.vgicp_voxel_resolution;
    auto reg = std::make_shared<SmallGicpRegistration>(cfg);
    reg->setScoreThreshold(config_.coarse_score_threshold);
    return reg;
  } else {
    // Default to NDT
    NdtConfig cfg;
    cfg.resolution = config_.ndt_resolution;
    cfg.max_iterations = config_.coarse_max_iterations;
    cfg.num_threads = config_.num_threads;
    auto reg = std::make_shared<NdtRegistration>(cfg);
    reg->setScoreThreshold(config_.coarse_score_threshold);
    return reg;
  }
}

RegistrationBase::Ptr CascadeRegistration::createFineRegistration() {
  if (config_.fine_method == "gicp" || config_.fine_method == "GICP") {
    GicpConfig cfg;
    cfg.max_iterations = config_.fine_max_iterations;
    cfg.max_correspondence_distance = config_.max_correspondence_distance;
    return std::make_shared<GicpRegistration>(cfg);
  } else {
    // Default to small_gicp
    SmallGicpConfig cfg;
    cfg.type = "GICP";
    cfg.max_iterations = config_.fine_max_iterations;
    cfg.num_threads = config_.num_threads;
    cfg.num_neighbors = config_.num_neighbors;
    cfg.max_correspondence_distance = config_.max_correspondence_distance;
    return std::make_shared<SmallGicpRegistration>(cfg);
  }
}

void CascadeRegistration::setTarget(const PointCloud::Ptr& target) {
  target_ = target;
  coarse_registration_->setTarget(target);
  fine_registration_->setTarget(target);
}

void CascadeRegistration::setSource(const PointCloud::Ptr& source) {
  source_ = source;
  coarse_registration_->setSource(source);
  fine_registration_->setSource(source);
}

RegistrationResult CascadeRegistration::align(const Eigen::Matrix4f& initial_guess) {
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

  std::cout << "========== Cascade Registration ==========" << std::endl;
  std::cout << "  Source: " << source_->size() << " points" << std::endl;
  std::cout << "  Target: " << target_->size() << " points" << std::endl;

  // ========== 第一阶段：粗配准 ==========
  std::cout << "\n[Stage 1] Coarse alignment: " << coarse_registration_->getName() << std::endl;

  auto coarse_result = coarse_registration_->align(initial_guess);

  std::cout << "  Converged: " << (coarse_result.has_converged ? "YES" : "NO") << std::endl;
  std::cout << "  Fitness score: " << coarse_result.fitness_score << std::endl;

  if (!coarse_result.has_converged) {
    result.message = "Coarse registration failed to converge";
    result.has_converged = false;
    return result;
  }

  // 即使 coarse score 较高也继续，因为精配准可能会改善
  Eigen::Matrix4f coarse_transform = coarse_result.transform;

  // ========== 第二阶段：精配准 ==========
  std::cout << "\n[Stage 2] Fine alignment: " << fine_registration_->getName() << std::endl;

  // 使用粗配准结果作为精配准的初始值
  fine_registration_->setScoreThreshold(score_threshold_);
  auto fine_result = fine_registration_->align(coarse_transform);

  std::cout << "  Converged: " << (fine_result.has_converged ? "YES" : "NO") << std::endl;
  std::cout << "  Fitness score: " << fine_result.fitness_score << std::endl;

  // ========== 结果评估 ==========
  // 采用两级阈值策略判断成功：
  //   1. 绝对阈值：fitness_score < score_threshold_ -> 直接成功 (SUCCESS)
  //   2. 相对改进：即使 fitness_score >= score_threshold_，只要精配准收敛且
  //      比粗配准有改进 (fine_score < coarse_score)，也算成功 (SUCCESS improved)
  //
  // 设计理由：实际场景中，点云质量、遮挡、动态物体等因素可能导致无法达到理想阈值，
  // 但只要配准结果相比初始猜测有显著改善，仍然是有价值的结果。
  result.has_converged = fine_result.has_converged;
  result.fitness_score = fine_result.fitness_score;
  result.transform = fine_result.transform;
  result.aligned_cloud = fine_result.aligned_cloud;

  if (fine_result.success) {
    // 情况1：精配准直接成功 (fitness_score < score_threshold_)
    result.success = true;
    result.message = "Cascade registration successful";
    std::cout << "\n[Cascade] SUCCESS" << std::endl;
  } else if (fine_result.has_converged && coarse_result.success) {
    // 情况2：精配准收敛但 fitness_score >= score_threshold_，检查相对改进
    if (fine_result.fitness_score < coarse_result.fitness_score) {
      // 精配准比粗配准更好，接受结果
      result.success = true;
      result.message = "Cascade registration successful (improved from coarse)";
      std::cout << "\n[Cascade] SUCCESS (improved)" << std::endl;
    } else {
      // 精配准没有改善，拒绝结果
      result.message = "Fine registration did not improve: " + std::to_string(fine_result.fitness_score);
      std::cout << "\n[Cascade] FAILED - no improvement" << std::endl;
    }
  } else {
    // 情况3：精配准未收敛或粗配准失败
    result.message = fine_result.message;
    std::cout << "\n[Cascade] FAILED - " << result.message << std::endl;
  }

  // 计算总变换量
  Eigen::Matrix4f delta = initial_guess.inverse() * result.transform;
  Eigen::Vector3f translation = delta.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = delta.block<3, 3>(0, 0);
  Eigen::AngleAxisf angle_axis(rotation);

  std::cout << "\n[Summary]" << std::endl;
  std::cout << "  Total translation: " << translation.norm() << " m" << std::endl;
  std::cout << "  Total rotation: " << angle_axis.angle() * 180.0 / M_PI << " deg" << std::endl;
  std::cout << "==========================================" << std::endl;

  return result;
}

}  // namespace finder_localization
