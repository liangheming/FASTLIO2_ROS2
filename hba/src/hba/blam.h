#pragma once
#include <cstdint>
#include <Eigen/Eigen>
#include <unordered_map>
#include <sophus/so3.hpp>

#include "commons.h"

#define HASH_P 116101
#define MAX_N 10000000000

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M3D r;
    V3D t;
};

class VoxelKey
{
public:
    int64_t x, y, z;
    VoxelKey(int64_t _x = 0, int64_t _y = 0, int64_t _z = 0) : x(_x), y(_y), z(_z) {}

    static VoxelKey index(double x, double y, double z, double resolution, double bias = 0.0);
    bool operator==(const VoxelKey &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
    struct Hasher
    {
        int64_t operator()(const VoxelKey &k) const
        {
            return ((((k.z) * HASH_P) % MAX_N + (k.y)) * HASH_P) % MAX_N + (k.x);
        }
    };
};

class OctoTree
{
public:
    OctoTree(int layer, int max_layer, int min_point_num, double plane_thresh, double quater_len, const V3D &center);
    bool isPlane() { return m_is_plane; }
    bool isValid() { return m_is_valid; }
    double &quaterLength() { return m_quater_len; }
    V3D &center() { return m_center; }
    V3D &mean() { return m_mean; }
    V3D &eigenVal() { return m_eigen_val; }
    M3D &eigenVec() { return m_eigen_vec; }
    PointVec &points() { return m_points; }
    PointVec &mergedPoints() { return m_merged_points; }

    void insert(const PointType &point);

    void buildPlanes();

    bool splitPlanes();

    void doMergePoints();

    void getPlanes(Vec<OctoTree *> &planes);

    int planeCount();

    double updateByPose(const Vec<Pose> &poses);

    double evalByPose(const Vec<Pose> &poses);

    M3D fp(const V3D &p);
    V3D dp(const V3D &p);
    M3D dp2(const V3D &p1, const V3D &p2, bool equal);

private:
    int m_layer;
    int m_max_layer;
    int m_min_point_num;
    double m_plane_thresh;
    double m_quater_len;
    bool m_is_valid;
    bool m_is_plane;
    V3D m_center;
    V3D m_mean;
    V3D m_eigen_val;
    M3D m_eigen_vec;
    Vec<std::shared_ptr<OctoTree>> m_leaves;
    PointVec m_points;
    PointVec m_merged_points;
};

struct BLAMConfig
{
    double voxel_size = 1.0;
    int min_point_num = 10;
    int max_layer = 4;
    double plane_thresh = 0.01;
    size_t max_iter = 10;
};

using VoxelMap = std::unordered_map<VoxelKey, std::shared_ptr<OctoTree>, VoxelKey::Hasher>;

class BLAM
{
public:
    BLAM(const BLAMConfig &config);
    BLAMConfig &config() { return m_config; }
    VoxelMap &voxelMap() { return m_voxel_map; }
    Vec<Pose> &poses() { return m_poses; }
    Vec<OctoTree *> &planes() { return m_planes; }
    Eigen::MatrixXd &H() { return m_H; }
    Eigen::VectorXd &J() { return m_J; }

    void insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose);

    void addPoint(const PointType &point);

    void buildVoxels();

    void updateJaccAndHess();

    void optimize();

    double updatePlanesByPoses(const Vec<Pose> &poses);

    double evalPlanesByPoses(const Vec<Pose> &poses);

    int planeCount(bool with_sub_planes = false);

    static void plusDelta(Vec<Pose> &poses, const Eigen::VectorXd &x);

    pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalCloud();

private:
    BLAMConfig m_config;
    Vec<Pose> m_poses;
    VoxelMap m_voxel_map;
    Vec<OctoTree *> m_planes;
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr> m_clouds;
    Eigen::MatrixXd m_H;
    Eigen::VectorXd m_J;
};
