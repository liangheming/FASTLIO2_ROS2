#pragma once
#include <cstdint>
#include <Eigen/Eigen>
#include <unordered_map>

#include "commons.h"
#include <sophus/so3.hpp>

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
    V3D center;
    double quater_len;
    OctoTree(int layer, int max_layer, int min_point_num, double plane_thresh);
    void build();
    void insert(const PointType &point);
    void split();
    int planeCount();
    void getAllPlanes(Vec<OctoTree *> &planes);
    V3D m_mean;
    V3D m_eigen_val;
    M3D m_eigen_vec;
    bool isPlane() { return m_is_plane; }

    M3D Fp(const V3D &p);
    V3D dp(const V3D &p);
    M3D dpdp(const V3D &p1, const V3D &p2, bool equal);
    PointVec &points() { return m_points; }

private:
    bool m_is_plane;
    bool m_is_valid;
    int m_layer;
    int m_max_layer;
    int m_min_point_num;
    double m_plane_thresh;
    Vec<std::shared_ptr<OctoTree>> m_leaves;
    PointVec m_points;
};

struct Config
{
    int max_layer = 2;
    int min_point_num = 10;
    double voxel_size = 1.0;
    double plane_thresh = 0.01;
    size_t max_iter = 10;
};

using VoxelMap = std::unordered_map<VoxelKey, std::shared_ptr<OctoTree>, VoxelKey::Hasher>;
class BLAM
{
public:
    BLAM(const Config &config);

    void addCloudAndPose(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose);

    void addPoint(const PointType &point);
    VoxelMap &voxelMap() { return m_map; }

    void buildVoxels();

    int planeCount();

    void getAllPlanes(Vec<OctoTree *> &planes);

    void optimize();

    void mergePointVec(const PointVec &in, PointVec &out);

    void buildMatrix(Vec<OctoTree *> &planes, Vec<PointVec> &all_points, Eigen::MatrixXd &H, Eigen::VectorXd &J);

    double evalResidual(const Vec<OctoTree *> &planes, const Vec<Pose> &poses);

    double updatePlanes(const Vec<OctoTree *> &planes, const Vec<Pose> &poses);

    void updateMergedPoints(Vec<PointVec> &merged_points, const Vec<Pose> &poses);

    void updatePoses(Vec<Pose> &poses, Eigen::VectorXd &x);
    Vec<Pose> &poses() { return m_poses; }

private:
    Config m_config;
    VoxelMap m_map;
    Vec<Pose> m_poses;
    Vec<PointVec> m_clouds;
};