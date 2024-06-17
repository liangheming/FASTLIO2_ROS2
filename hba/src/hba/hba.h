#pragma once
#include "commons.h"
#include "blam.h"
#include <fstream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>


struct HBAConfig
{
    int window_size = 20;
    int stride = 10;
    double voxel_size = 1.0;
    int min_point_num = 10;
    int max_layer = 4;
    double plane_thresh = 0.01;
    size_t ba_max_iter = 10;
    size_t hba_iter = 2;
    double down_sample = 0.1;
};

class HBA
{
public:
    HBA(const HBAConfig config);
    void insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose);

    void optimize();

    int calcLevels();

    void constructHierarchy(Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, Vec<Pose> &poses, int level);

    void updateCloudsAndPose(Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds, Vec<Pose> &poses, int level);
    int levels() { return m_levels; }
    HBAConfig &config() { return m_config; }
    Vec<Pose> &poses() { return m_poses; }
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> &clouds() { return m_clouds; }

    void getAllFactors(Vec<std::pair<size_t, size_t>> &ids, Vec<Pose> &poses, Vec<M6D> &infos);

    pcl::PointCloud<pcl::PointXYZI>::Ptr getMapPoints();

    void writePoses(const std::string &path);

private:
    int m_levels = 1;
    HBAConfig m_config;
    Vec<Pose> m_poses;
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_clouds;
    Vec<Vec<BLAM>> m_lbas;
};
