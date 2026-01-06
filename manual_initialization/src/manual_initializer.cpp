// ============================================================
// Manual Initialization Optimizer - Implementation
//
// 功能：通过局部搜索优化手动给定的初始位姿
//
// Author: AI Assistant
// Date: 2025-12-27
// ============================================================

#include "manual_initializer.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>

namespace finder_localization {

// 静态辅助函数：点云降采样
static pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float voxel_size) {
  if (voxel_size <= 0.0f) {
    return cloud; // 不降采样
  }

  auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(cloud);
  voxel.filter(*filtered);
  return filtered;
}

// 静态辅助函数：评估单个候选位姿（单尺度）
static bool evaluatePose(pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> &ndt, const Eigen::Matrix4f &guess, double &score, Eigen::Matrix4f &final_transform) {
  pcl::PointCloud<pcl::PointXYZI> aligned;
  ndt.align(aligned, guess);

  if (ndt.hasConverged()) {
    score = ndt.getFitnessScore();
    final_transform = ndt.getFinalTransformation();
    return true;
  }

  return false;
}

ManualInitializer::ManualInitializer(const ManualInitializerConfig &config) : config_(config) {}

void ManualInitializer::updateConfig(const ManualInitializerConfig &config) { config_ = config; }

OptimizationResult ManualInitializer::optimize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud,
                                               const Eigen::Matrix4f &initial_guess) {
  OptimizationResult result;
  result.transform = initial_guess;

  // 1. 输入验证
  if (!source_cloud || source_cloud->empty()) {
    result.message = "Source cloud is empty";
    return result;
  }

  if (!target_cloud || target_cloud->empty()) {
    result.message = "Target cloud is empty";
    return result;
  }

  std::cout << "[ManualInitializer] Starting optimization..." << std::endl;
  std::cout << "  Source cloud: " << source_cloud->size() << " points" << std::endl;
  std::cout << "  Target cloud: " << target_cloud->size() << " points" << std::endl;

  // 2. 生成搜索网格
  auto search_grid = generateSearchGrid(initial_guess);
  std::cout << "  Search grid: " << search_grid.size() << " candidates" << std::endl;

  // 3. 搜索最佳位姿（使用多尺度或单尺度）
  double best_score = std::numeric_limits<double>::max();
  Eigen::Matrix4f best_transform = initial_guess;
  bool found = false;

  result.evaluated_poses = 0;
  result.converged_poses = 0;

  if (config_.use_multiscale) {
    // === 多尺度配准模式（优化版：预处理缓存）===
    std::cout << "  Using multi-scale registration (coarse->fine): ";
    for (double res : config_.multiscale_resolutions)
      std::cout << res << "m ";
    std::cout << std::endl;

    // 🔥 预处理：为每个尺度降采样并构建 NDT（只做一次！）
    std::cout << "  Preprocessing scales..." << std::endl;

    struct ScaleData {
      pcl::PointCloud<pcl::PointXYZI>::Ptr source;
      pcl::PointCloud<pcl::PointXYZI>::Ptr target;
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    };

    std::vector<ScaleData> scales(config_.multiscale_resolutions.size());

    // 🔧 动态生成体素大小：根据尺度数量自适应
    // 规则：粗尺度用大体素降采样，细尺度不降采样（0.0）
    std::vector<float> voxel_sizes;
    if (scales.size() == 1) {
      voxel_sizes = {0.0f}; // 单尺度：不降采样
    } else if (scales.size() == 2) {
      voxel_sizes = {0.3f, 0.0f}; // 粗、精
    } else if (scales.size() == 3) {
      voxel_sizes = {0.5f, 0.2f, 0.0f}; // 粗、中、精
    } else {
      // 多于3个尺度：前面降采样，最后不降采样
      for (size_t i = 0; i < scales.size() - 1; i++) {
        float voxel_size = 0.6f - i * 0.15f; // 0.6, 0.45, 0.3, 0.15...
        voxel_sizes.push_back(std::max(0.1f, voxel_size));
      }
      voxel_sizes.push_back(0.0f); // 最细尺度不降采样
    }

    // 🛡️ 安全检查：确保大小一致
    if (voxel_sizes.size() != scales.size()) {
      std::cerr << "[ManualInitializer] ERROR: voxel_sizes.size() != scales.size()" << std::endl;
      result.message = "Internal error: voxel size mismatch";
      return result;
    }

    for (size_t i = 0; i < scales.size(); i++) {
      // 降采样（只做一次）
      scales[i].source = downsampleCloud(source_cloud, voxel_sizes[i]);
      scales[i].target = downsampleCloud(target_cloud, voxel_sizes[i]);

      // 配置并构建 NDT（耗时操作，只做一次！）
      scales[i].ndt.setResolution(config_.multiscale_resolutions[i]);
      scales[i].ndt.setStepSize(config_.ndt_step_size);
      scales[i].ndt.setTransformationEpsilon(config_.ndt_transform_epsilon);
      scales[i].ndt.setMaximumIterations(config_.multiscale_iterations[i]);
      scales[i].ndt.setNumThreads(config_.num_threads);
      scales[i].ndt.setInputTarget(scales[i].target); // 🔥 构建体素结构，只做一次
      scales[i].ndt.setInputSource(scales[i].source);

      std::cout << "    Scale " << i << ": " << scales[i].source->size() << " src pts, " << scales[i].target->size() << " tgt pts" << std::endl;
    }

    // 遍历候选位姿
    for (const auto &guess : search_grid) {
      result.evaluated_poses++;

      // 🔥 多尺度配准：逐尺度优化（粗->中->精）
      Eigen::Matrix4f current_guess = guess;
      bool converged = true;
      double final_score = 0.0;

      for (size_t scale = 0; scale < scales.size(); scale++) {
        pcl::PointCloud<pcl::PointXYZI> aligned;
        scales[scale].ndt.align(aligned, current_guess);

        if (!scales[scale].ndt.hasConverged()) {
          converged = false;
          break;
        }

        current_guess = scales[scale].ndt.getFinalTransformation();
        final_score = scales[scale].ndt.getFitnessScore();
      }

      if (converged) {
        result.converged_poses++;

        if (final_score < best_score) {
          best_score = final_score;
          best_transform = current_guess;
          found = true;

          // 🔥 早停：如果得分足够好，提前退出
          if (config_.early_stop_score > 0.0 && best_score < config_.early_stop_score) {
            std::cout << "  ⚡ Early stop: best score " << best_score << " < threshold " << config_.early_stop_score << std::endl;
            break;
          }
        }
      }

      // 定期输出进度
      if (result.evaluated_poses % 50 == 0) {
        std::cout << "  Progress: " << result.evaluated_poses << "/" << search_grid.size() << ", converged: " << result.converged_poses << ", best score: " << best_score << std::endl;
      }
    }
  } else {
    // === 单尺度配准模式 ===
    std::cout << "  Using single-scale registration: " << config_.ndt_resolution << "m" << std::endl;

    auto ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>();
    ndt.setResolution(config_.ndt_resolution);
    ndt.setStepSize(config_.ndt_step_size);
    ndt.setTransformationEpsilon(config_.ndt_transform_epsilon);
    ndt.setMaximumIterations(config_.ndt_max_iterations);
    ndt.setNumThreads(config_.num_threads);
    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(source_cloud);

    for (const auto &guess : search_grid) {
      double score;
      Eigen::Matrix4f final_transform;

      result.evaluated_poses++;

      if (evaluatePose(ndt, guess, score, final_transform)) {
        result.converged_poses++;

        if (score < best_score) {
          best_score = score;
          best_transform = final_transform;
          found = true;

          // 🔥 早停：如果得分足够好，提前退出
          if (config_.early_stop_score > 0.0 && best_score < config_.early_stop_score) {
            std::cout << "  ⚡ Early stop: best score " << best_score << " < threshold " << config_.early_stop_score << std::endl;
            break;
          }
        }
      }

      // 定期输出进度
      if (result.evaluated_poses % 50 == 0) {
        std::cout << "  Progress: " << result.evaluated_poses << "/" << search_grid.size() << ", converged: " << result.converged_poses << ", best score: " << best_score << std::endl;
      }
    }
  }

  // 5. 结果处理
  if (!found) {
    result.message = "No valid alignment found";
    return result;
  }

  result.transform = best_transform;
  result.fitness_score = best_score;

  // 6. 计算改进量
  computeImprovement(initial_guess, best_transform, result.position_improvement, result.yaw_improvement);

  // 7. 验证结果
  if (!validateResult(result)) {
    result.message = "Refinement exceeds safety limits";
    return result;
  }

  result.success = true;
  result.message = "Optimization successful";

  // 8. 生成对齐后的点云
  result.aligned_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::transformPointCloud(*source_cloud, *result.aligned_cloud, best_transform);

  std::cout << "[ManualInitializer] Optimization completed:" << std::endl;
  std::cout << "  Evaluated: " << result.evaluated_poses << ", Converged: " << result.converged_poses << std::endl;
  std::cout << "  Best score: " << result.fitness_score << std::endl;
  std::cout << "  Position improvement: " << result.position_improvement << " m" << std::endl;
  std::cout << "  Yaw improvement: " << result.yaw_improvement * 180.0 / M_PI << " deg" << std::endl;

  return result;
}

std::vector<Eigen::Matrix4f> ManualInitializer::generateSearchGrid(const Eigen::Matrix4f &center) {
  std::vector<Eigen::Matrix4f> grid;

  // 提取中心位置和姿态
  Eigen::Vector3f center_pos = center.block<3, 1>(0, 3);
  double center_yaw = extractYaw(center);

  // 🔥 根据 search_radius 动态生成同心圆半径
  std::vector<double> radii;
  std::vector<int> num_angles;

  // 中心点
  radii.push_back(0.0);
  num_angles.push_back(1);

  // 动态生成同心圆（步长 0.5m）
  double step = 0.5;
  for (double r = step; r <= config_.search_radius + 0.01; r += step) {
    radii.push_back(r);
    // 角度采样数量随半径增加
    int n_angles_for_radius = std::max(8, static_cast<int>(r * 8));
    num_angles.push_back(n_angles_for_radius);
  }

  // Yaw 角度采样
  std::vector<double> yaw_offsets = {0.0, -config_.yaw_search_range * 0.5, config_.yaw_search_range * 0.5, -config_.yaw_search_range, config_.yaw_search_range};

  for (size_t r_idx = 0; r_idx < radii.size(); r_idx++) {
    double radius = radii[r_idx];
    int n_angles = num_angles[r_idx];

    for (int a_idx = 0; a_idx < n_angles; a_idx++) {
      double angle = 2.0 * M_PI * a_idx / n_angles;

      // XY 偏移
      double dx = radius * std::cos(angle);
      double dy = radius * std::sin(angle);

      // 对每个 XY 位置，尝试多个航向角
      for (double yaw_offset : yaw_offsets) {
        Eigen::Matrix4f candidate = Eigen::Matrix4f::Identity();

        // 设置旋转（只有 yaw）
        double yaw = center_yaw + yaw_offset;
        candidate(0, 0) = std::cos(yaw);
        candidate(0, 1) = -std::sin(yaw);
        candidate(1, 0) = std::sin(yaw);
        candidate(1, 1) = std::cos(yaw);

        // 设置位置
        candidate(0, 3) = center_pos.x() + dx;
        candidate(1, 3) = center_pos.y() + dy;
        candidate(2, 3) = center_pos.z(); // Z 保持不变

        grid.push_back(candidate);
      }
    }
  }

  return grid;
}

void ManualInitializer::computeImprovement(const Eigen::Matrix4f &initial, const Eigen::Matrix4f &refined, double &position_improvement, double &yaw_improvement) {
  // 位置改进
  Eigen::Vector3f pos_initial = initial.block<3, 1>(0, 3);
  Eigen::Vector3f pos_refined = refined.block<3, 1>(0, 3);
  Eigen::Vector3f pos_diff = pos_refined - pos_initial;
  position_improvement = pos_diff.norm();

  // 角度改进
  double yaw_initial = extractYaw(initial);
  double yaw_refined = extractYaw(refined);
  yaw_improvement = std::abs(yaw_refined - yaw_initial);

  // 处理角度环绕（-π 到 π）
  if (yaw_improvement > M_PI) {
    yaw_improvement = 2.0 * M_PI - yaw_improvement;
  }
}

double ManualInitializer::extractYaw(const Eigen::Matrix4f &transform) {
  // 从旋转矩阵提取 yaw 角
  // R = [cos(yaw)  -sin(yaw)  0]
  //     [sin(yaw)   cos(yaw)  0]
  //     [0          0         1]

  return std::atan2(transform(1, 0), transform(0, 0));
}

bool ManualInitializer::validateResult(const OptimizationResult &result) {
  // 检查位置改进是否在合理范围内
  if (result.position_improvement > config_.max_refinement_dist) {
    std::cout << "[ManualInitializer] WARNING: Position improvement " << result.position_improvement << " m exceeds limit " << config_.max_refinement_dist << " m" << std::endl;
    return false;
  }

  // 检查角度改进是否在合理范围内
  if (result.yaw_improvement > config_.max_refinement_yaw) {
    std::cout << "[ManualInitializer] WARNING: Yaw improvement " << result.yaw_improvement * 180.0 / M_PI << " deg exceeds limit " << config_.max_refinement_yaw * 180.0 / M_PI << " deg" << std::endl;
    return false;
  }

  return true;
}

OptimizationResult ManualInitializer::simpleOptimize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud,
                                                      const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud,
                                                      const Eigen::Matrix4f &initial_guess) {
  OptimizationResult result;
  result.transform = initial_guess;

  // 1. 输入验证
  if (!source_cloud || source_cloud->empty()) {
    result.message = "Source cloud is empty";
    return result;
  }

  if (!target_cloud || target_cloud->empty()) {
    result.message = "Target cloud is empty";
    return result;
  }

  std::cout << "[ManualInitializer] Simple optimization (direct NDT)..." << std::endl;
  std::cout << "  Source cloud: " << source_cloud->size() << " points" << std::endl;
  std::cout << "  Target cloud: " << target_cloud->size() << " points" << std::endl;
  std::cout << "  Initial guess: [" << initial_guess(0, 3) << ", " << initial_guess(1, 3) << ", " << initial_guess(2, 3) << "]" << std::endl;

  // 2. 配置 NDT
  pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setResolution(config_.ndt_resolution);
  ndt.setStepSize(config_.ndt_step_size);
  ndt.setTransformationEpsilon(config_.ndt_transform_epsilon);
  ndt.setMaximumIterations(config_.ndt_max_iterations);
  ndt.setNumThreads(config_.num_threads);

  // 3. 设置输入
  ndt.setInputTarget(target_cloud);
  ndt.setInputSource(source_cloud);

  // 4. 执行配准
  pcl::PointCloud<pcl::PointXYZI> aligned;
  ndt.align(aligned, initial_guess);

  // 5. 检查结果
  result.evaluated_poses = 1;
  result.has_converged = ndt.hasConverged();
  result.fitness_score = ndt.getFitnessScore();

  if (result.has_converged) {
    result.converged_poses = 1;
    result.transform = ndt.getFinalTransformation();

    // 计算改进量
    computeImprovement(initial_guess, result.transform, result.position_improvement, result.yaw_improvement);

    // 生成对齐后的点云
    result.aligned_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*source_cloud, *result.aligned_cloud, result.transform);

    std::cout << "[ManualInitializer] Simple optimization completed:" << std::endl;
    std::cout << "  Converged: YES" << std::endl;
    std::cout << "  Fitness score: " << result.fitness_score << " (threshold: " << config_.score_threshold << ")" << std::endl;
    std::cout << "  Final pose: [" << result.transform(0, 3) << ", " << result.transform(1, 3) << ", " << result.transform(2, 3) << "]" << std::endl;
    std::cout << "  Position improvement: " << result.position_improvement << " m" << std::endl;
    std::cout << "  Yaw improvement: " << result.yaw_improvement * 180.0 / M_PI << " deg" << std::endl;

    // 检查 fitness score 是否满足阈值
    if (result.fitness_score < config_.score_threshold) {
      result.success = true;
      result.message = "Simple optimization successful";
      std::cout << "  Result: SUCCESS (score < threshold)" << std::endl;
    } else {
      result.success = false;
      result.message = "Fitness score too high: " + std::to_string(result.fitness_score) + " >= " + std::to_string(config_.score_threshold);
      std::cout << "  Result: FAILED (score >= threshold)" << std::endl;
    }
  } else {
    result.converged_poses = 0;
    result.message = "NDT failed to converge";
    std::cout << "[ManualInitializer] Simple optimization FAILED: NDT did not converge" << std::endl;
  }

  return result;
}

} // namespace finder_localization
