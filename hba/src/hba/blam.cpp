#include "blam.h"

VoxelKey VoxelKey::index(double x, double y, double z, double resolution, double bias)
{
    V3D point(x, y, z);
    V3D idx = (point / resolution + V3D(bias, bias, bias)).array().floor();
    return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
}
OctoTree::OctoTree(int layer, int max_layer, int min_point_num, double plane_thresh) : m_is_plane(false), m_is_valid(false), m_layer(layer), m_max_layer(max_layer), m_min_point_num(min_point_num), m_plane_thresh(plane_thresh)
{
    m_leaves.resize(8, nullptr);
}
int OctoTree::planeCount()
{
    if (!m_is_valid)
        return 0;
    if (m_is_plane)
        return 1;
    int count = 0;
    if (m_layer < m_max_layer - 1)
    {
        for (auto &leaf : m_leaves)
        {
            if (leaf != nullptr)
                count += leaf->planeCount();
        }
    }
    return count;
}
void OctoTree::split()
{
    if (m_layer >= m_max_layer - 1)
        return;
    for (PointType &point : m_points)
    {
        int xyz[3] = {0, 0, 0};
        if (point.x > center.x())
            xyz[0] = 1;
        if (point.y > center.y())
            xyz[1] = 1;
        if (point.z > center.z())
            xyz[2] = 1;
        int leaf_num = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

        if (m_leaves[leaf_num] == nullptr)
        {
            m_leaves[leaf_num].reset(new OctoTree(m_layer + 1, m_max_layer, m_min_point_num, m_plane_thresh));
            V3D shift((2 * xyz[0] - 1) * quater_len, (2 * xyz[1] - 1) * quater_len, (2 * xyz[2] - 1) * quater_len);
            m_leaves[leaf_num]->center = center + shift;
            m_leaves[leaf_num]->quater_len = quater_len / 2;
        }
        m_leaves[leaf_num]->insert(point);
    }

    PointVec().swap(m_points);

    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
        {
            leaf->build();
        }
    }
}
void OctoTree::build()
{
    if (m_points.size() < m_min_point_num)
    {
        m_is_plane = false;
        m_is_valid = false;
        return;
    }

    m_is_valid = true;
    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();

    for (PointType &point : m_points)
    {
        V3D p_vec(point.x, point.y, point.z);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);

    V3D eigen_val = eigensolver.eigenvalues();
    // M3D eigen_vec = eigensolver.eigenvectors();
    // std::cout << eigen_val.transpose() << std::endl;
    if (eigen_val[0] < m_plane_thresh)
    {
        m_is_plane = true;
        return;
    }
    m_is_plane = false;
    split();
}
void OctoTree::insert(const PointType &point)
{
    m_points.push_back(point);
}
BLAM::BLAM(const Config &config) : m_config(config)
{
    m_map.clear();
    m_poses.clear();
    if (m_config.scan_resolution > 0.0)
    {
        m_voxel_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }
}
void BLAM::addPoint(const PointType &point)
{
    VoxelKey loc = VoxelKey::index(point.x, point.y, point.z, m_config.voxel_size, 0.0);
    if (m_map.find(loc) == m_map.end())
    {
        m_map[loc] = std::make_shared<OctoTree>(0, m_config.max_layer, m_config.min_point_num, m_config.plane_thresh);
        m_map[loc]->center = V3D((0.5 + loc.x) * m_config.voxel_size, (0.5 + loc.y) * m_config.voxel_size, (0.5 + loc.z) * m_config.voxel_size);
        m_map[loc]->quater_len = m_config.voxel_size / 4.0;
    }
    m_map[loc]->insert(point);
}
void BLAM::addCloudAndPose(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    int idx = m_poses.size();
    m_poses.push_back(pose);

    if (m_config.scan_resolution > 0.0)
    {
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*cloud);
    }
    for (auto &p : cloud->points)
    {
        V3D p_vec(p.x, p.y, p.z);
        p_vec = pose.r * p_vec + pose.t;
        PointType point;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        point.lx = p.x;
        point.ly = p.y;
        point.lz = p.z;
        point.intensity = p.intensity;
        point.id = idx;
        point.time = 0.0;
        addPoint(point);
    }
}

void BLAM::buildVoxels()
{
    for (auto &p : m_map)
        p.second->build();
}

int BLAM::planeCount()
{
    int count = 0;
    for (auto &p : m_map)
    {
        count += p.second->planeCount();
    }
    return count;
}