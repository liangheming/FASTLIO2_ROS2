// ============================================================
// Manual Initialization Optimizer - Header
//
// 功能：通过局部搜索优化手动给定的初始位姿
//
// Author: AI Assistant
// Date: 2025-12-27
// ============================================================

#ifndef MANUAL_INITIALIZER_HPP
#define MANUAL_INITIALIZER_HPP

#include <Eigen/Dense>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utility>
#include <vector>

namespace finder_localization {

/**
 * @brief 手动初始化优化配置
 */
struct ManualInitializerConfig {
  // 搜索范围参数
  double search_radius = 2.0;          // XY 平面搜索半径（米）
  double yaw_search_range = 0.7853975; // 航向角搜索范围（弧度，±90度）

  // NDT 配准参数
  bool use_multiscale = true;          // 是否使用多尺度配准（推荐）
  double ndt_resolution = 1.5;         // NDT 体素分辨率（单尺度模式使用）
  double ndt_step_size = 0.1;          // 优化步长
  double ndt_transform_epsilon = 0.01; // 变换收敛阈值
  int ndt_max_iterations = 20;         // 最大迭代次数（单尺度模式使用）
  int num_threads = 4;                 // 并行线程数

  // 多尺度配准参数（use_multiscale=true 时使用）
  std::vector<double> multiscale_resolutions = {3.0, 1.5, 1.0}; // 粗->中->精
  std::vector<int> multiscale_iterations = {15, 20, 25};        // 各尺度迭代次数

  // 验证参数
  double max_refinement_dist = 2.5;  // 允许的最大位置改进距离（米）
  double max_refinement_yaw = 0.524; // 允许的最大角度改进（弧度，30度）

  // 早停参数
  double early_stop_score = 0.1; // 提前退出阈值（0=禁用，>0时得分低于此值则提前退出）

  // 配准成功阈值
  double score_threshold = 0.15; // fitness score 阈值，低于此值认为配准成功
};

/**
 * @brief 优化结果
 */
struct OptimizationResult {
  bool success = false;       // 是否成功
  bool has_converged = false; // NDT 是否收敛
  Eigen::Matrix4f transform;  // 优化后的变换矩阵
  double fitness_score = 0.0; // 配准得分
  int evaluated_poses = 0;    // 评估的位姿数量
  int converged_poses = 0;    // 收敛的位姿数量

  // 改进量统计
  double position_improvement = 0.0; // 位置改进（米）
  double yaw_improvement = 0.0;      // 角度改进（弧度）

  std::string message; // 状态消息

  // 对齐后的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_cloud;
};

/**
 * @brief 手动初始化优化器
 *
 * 在用户手动给定的初始位姿周围进行局部搜索，
 * 找到配准得分最好的位置作为优化后的初始位姿
 */
class ManualInitializer {
public:
  /**
   * @brief 构造函数
   * @param config 配置参数
   */
  explicit ManualInitializer(const ManualInitializerConfig &config);

  /**
   * @brief 析构函数
   */
  ~ManualInitializer() = default;

  /**
   * @brief 优化初始位姿（带搜索网格）
   *
   * @param source_cloud 当前扫描点云
   * @param target_cloud 局部地图点云
   * @param initial_guess 初始猜测位姿（4x4 变换矩阵）
   * @return OptimizationResult 优化结果
   */
  OptimizationResult optimize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, const Eigen::Matrix4f &initial_guess);

  /**
   * @brief 简单优化初始位姿（直接 NDT 配准，无搜索）
   *
   * 直接使用 initial_guess 进行 NDT 配准，用于快速验证算法流程
   *
   * @param source_cloud 当前扫描点云（lidar frame）
   * @param target_cloud 局部地图点云（map frame）
   * @param initial_guess 初始猜测位姿 T_map_lidar（4x4 变换矩阵）
   * @return OptimizationResult 优化结果
   */
  OptimizationResult simpleOptimize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, const Eigen::Matrix4f &initial_guess);

  /**
   * @brief 更新配置参数
   * @param config 新的配置参数
   */
  void updateConfig(const ManualInitializerConfig &config);

  /**
   * @brief 获取当前配置
   * @return 当前配置参数
   */
  const ManualInitializerConfig &getConfig() const { return config_; }

private:
  /**
   * @brief 生成搜索网格
   *
   * 以初始位姿为中心，生成同心圆网格上的候选位姿
   *
   * @param center 中心位姿
   * @return 候选位姿列表（变换矩阵）
   */
  std::vector<Eigen::Matrix4f> generateSearchGrid(const Eigen::Matrix4f &center);

  /**
   * @brief 计算位姿改进量
   *
   * @param initial 初始位姿
   * @param refined 优化后位姿
   * @param position_improvement 输出：位置改进（米）
   * @param yaw_improvement 输出：角度改进（弧度）
   */
  void computeImprovement(const Eigen::Matrix4f &initial, const Eigen::Matrix4f &refined, double &position_improvement, double &yaw_improvement);

  /**
   * @brief 从变换矩阵提取 yaw 角
   * @param transform 变换矩阵
   * @return yaw 角（弧度）
   */
  double extractYaw(const Eigen::Matrix4f &transform);

  /**
   * @brief 验证优化结果的合理性
   *
   * @param result 优化结果
   * @return 是否通过验证
   */
  bool validateResult(const OptimizationResult &result);

private:
  ManualInitializerConfig config_; // 配置参数
};

} // namespace finder_localization

#endif // MANUAL_INITIALIZER_HPP
