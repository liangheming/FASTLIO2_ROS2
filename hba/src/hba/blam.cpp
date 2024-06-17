#include "blam.h"
VoxelKey VoxelKey::index(double x, double y, double z, double resolution, double bias)
{
    V3D point(x, y, z);
    V3D idx = (point / resolution + V3D(bias, bias, bias)).array().floor();
    return VoxelKey(static_cast<int64_t>(idx(0)), static_cast<int64_t>(idx(1)), static_cast<int64_t>(idx(2)));
}
OctoTree::OctoTree(int layer, int max_layer, int min_point_num, double plane_thresh, double quater_len, const V3D &center)
    : m_layer(layer), m_max_layer(max_layer), m_min_point_num(min_point_num), m_plane_thresh(plane_thresh), m_quater_len(quater_len), m_is_valid(false), m_is_plane(false), m_center(center)
{
    m_leaves.resize(8, nullptr);
}
void OctoTree::insert(const PointType &point)
{
    m_points.push_back(point);
}
void OctoTree::buildPlanes()
{
    if (static_cast<int>(m_points.size()) < m_min_point_num)
    {
        m_is_plane = false;
        m_is_valid = false;
        return;
    }

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
    m_mean = mean;
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
    if (m_eigen_val[0] < m_plane_thresh)
    {
        m_mean = mean;
        m_is_plane = true;
        m_is_valid = true;
        doMergePoints();
        return;
    }
    m_is_plane = false;
    m_is_valid = splitPlanes();
}
void OctoTree::doMergePoints()
{
    assert(m_is_valid && m_is_plane);
    std::unordered_map<uint32_t, PointType> id_point_map;
    for (PointType &point : m_points)
    {
        uint32_t id = point.id;
        if (id_point_map.find(id) == id_point_map.end())
        {
            id_point_map[id] = point;
            id_point_map[id].time = 1.0;
        }
        else
        {
            id_point_map[id].x += point.x;
            id_point_map[id].y += point.y;
            id_point_map[id].z += point.z;
            id_point_map[id].intensity += point.intensity;
            id_point_map[id].lx += point.lx;
            id_point_map[id].ly += point.ly;
            id_point_map[id].lz += point.lz;
            id_point_map[id].time += 1.0;
        }
    }

    PointVec().swap(m_merged_points);
    for (auto &iter : id_point_map)
    {
        PointType &point = iter.second;
        point.x /= point.time;
        point.y /= point.time;
        point.z /= point.time;
        point.intensity /= point.time;
        point.lx /= point.time;
        point.ly /= point.time;
        point.lz /= point.time;
        point.time = 1.0;
        m_merged_points.push_back(point);
    }
}
bool OctoTree::splitPlanes()
{
    if (m_layer >= m_max_layer - 1)
        return false;
    for (PointType &point : m_points)
    {
        int xyz[3] = {0, 0, 0};
        if (point.x > m_center.x())
            xyz[0] = 1;
        if (point.y > m_center.y())
            xyz[1] = 1;
        if (point.z > m_center.z())
            xyz[2] = 1;

        int leaf_num = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (m_leaves[leaf_num] == nullptr)
        {
            V3D shift((2 * xyz[0] - 1) * m_quater_len, (2 * xyz[1] - 1) * m_quater_len, (2 * xyz[2] - 1) * m_quater_len);
            V3D center = m_center + shift;
            m_leaves[leaf_num].reset(new OctoTree(m_layer + 1, m_max_layer, m_min_point_num, m_plane_thresh, m_quater_len / 2.0, center));
        }
        m_leaves[leaf_num]->insert(point);
    }

    PointVec().swap(m_points);
    PointVec().swap(m_merged_points);

    bool has_sub_plane = false;
    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
        {
            leaf->buildPlanes();
            if (leaf->isValid())
                has_sub_plane = true;
        }
    }
    return has_sub_plane;
}
void OctoTree::getPlanes(Vec<OctoTree *> &planes)
{
    if (m_is_plane)
    {
        planes.push_back(this);
        return;
    }
    if (!m_is_valid)
        return;

    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
            leaf->getPlanes(planes);
    }
}
int OctoTree::planeCount()
{
    if (m_is_plane)
        return 1;

    if (!m_is_valid)
        return 0;

    int count = 0;
    for (auto &leaf : m_leaves)
    {
        if (leaf != nullptr)
            count += leaf->planeCount();
    }
    return count;
}
double OctoTree::updateByPose(const Vec<Pose> &poses)
{
    assert(m_is_valid && m_is_plane);
    for (PointType &point : m_merged_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
    }

    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    m_mean = mean;
    m_eigen_val = eigensolver.eigenvalues();
    m_eigen_vec = eigensolver.eigenvectors();
    return m_eigen_val[0];
}
double OctoTree::evalByPose(const Vec<Pose> &poses)
{
    assert(m_is_valid && m_is_plane);
    V3D mean = V3D::Zero();
    M3D cov = M3D::Zero();
    for (PointType &point : m_points)
    {
        V3D p_vec(point.lx, point.ly, point.lz);
        const Pose &pose = poses[point.id];
        p_vec = pose.r * p_vec + pose.t;
        point.x = p_vec(0);
        point.y = p_vec(1);
        point.z = p_vec(2);
        mean += p_vec;
        cov += p_vec * p_vec.transpose();
    }
    mean /= static_cast<double>(m_points.size());
    cov = cov / static_cast<double>(m_points.size()) - mean * mean.transpose();
    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    return eigensolver.eigenvalues()[0];
}
M3D OctoTree::fp(const V3D &p)
{
    M3D ret = M3D::Zero();
    ret.row(1) = (p - m_mean).transpose() * (m_eigen_vec.col(1) * m_eigen_vec.col(0).transpose() + m_eigen_vec.col(0) * m_eigen_vec.col(1).transpose()) / static_cast<double>(m_points.size()) / (m_eigen_val(0) - m_eigen_val(1));
    ret.row(2) = (p - m_mean).transpose() * (m_eigen_vec.col(2) * m_eigen_vec.col(0).transpose() + m_eigen_vec.col(0) * m_eigen_vec.col(2).transpose()) / static_cast<double>(m_points.size()) / (m_eigen_val(0) - m_eigen_val(2));
    return ret;
}
V3D OctoTree::dp(const V3D &p)
{
    return 2.0 / static_cast<double>(m_points.size()) * ((p - m_mean).transpose() * m_eigen_vec.col(0) * m_eigen_vec.col(0).transpose()).transpose();
}
M3D OctoTree::dp2(const V3D &p1, const V3D &p2, bool equal)
{
    double n = static_cast<double>(m_points.size());
    V3D u0 = m_eigen_vec.col(0);
    if (equal)
    {
        return 2.0 / n * ((n - 1) / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * fp(p2) + m_eigen_vec * fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
    else
    {
        return 2.0 / n * (-1.0 / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * fp(p2) + m_eigen_vec * fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
}
BLAM::BLAM(const BLAMConfig &config) : m_config(config)
{
    Vec<Pose>().swap(m_poses);
    Vec<pcl::PointCloud<pcl::PointXYZI>::Ptr>().swap(m_clouds);
}
void BLAM::insert(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const Pose &pose)
{
    m_poses.push_back(pose);
    m_clouds.push_back(cloud);
}

void BLAM::addPoint(const PointType &point)
{
    VoxelKey loc = VoxelKey::index(point.x, point.y, point.z, m_config.voxel_size, 0.0);
    if (m_voxel_map.find(loc) == m_voxel_map.end())
    {
        V3D center = V3D((0.5 + loc.x) * m_config.voxel_size, (0.5 + loc.y) * m_config.voxel_size, (0.5 + loc.z) * m_config.voxel_size);
        double quater_len = m_config.voxel_size / 4.0;
        m_voxel_map[loc] = std::make_shared<OctoTree>(0, m_config.max_layer, m_config.min_point_num, m_config.plane_thresh, quater_len, center);
    }
    m_voxel_map[loc]->insert(point);
}
void BLAM::buildVoxels()
{

    Vec<OctoTree *>().swap(m_planes);
    VoxelMap().swap(m_voxel_map);
    for (size_t i = 0; i < m_clouds.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = m_clouds[i];
        const Pose &pose = m_poses[i];
        for (pcl::PointXYZI &point : cloud->points)
        {
            PointType p;
            V3D p_vec(point.x, point.y, point.z);
            p_vec = pose.r * p_vec + pose.t;
            p.x = static_cast<float>(p_vec.x());
            p.y = static_cast<float>(p_vec.y());
            p.z = static_cast<float>(p_vec.z());
            p.lx = point.x;
            p.ly = point.y;
            p.lz = point.z;
            p.intensity = point.intensity;
            p.id = static_cast<uint32_t>(i);
            p.time = 1.0;
            addPoint(p);
        }
    }

    for (auto &iter : m_voxel_map)
    {
        iter.second->buildPlanes();
        iter.second->getPlanes(m_planes);
    }
}
void BLAM::updateJaccAndHess()
{
    m_H.setZero();
    m_J.setZero();
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane_ptr = m_planes[i];
        const PointVec &points = plane_ptr->mergedPoints();
        Eigen::MatrixXd D_i = Eigen::MatrixXd::Zero(3 * points.size(), 6 * points.size());
        Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(3 * points.size(), 3 * points.size());
        Eigen::MatrixXd J_i = Eigen::MatrixXd::Zero(1, 3 * points.size());
        Vec<uint32_t> idxs(points.size(), 0);
        for (size_t m = 0; m < points.size(); m++)
        {
            const PointType &point1 = points[m];
            V3D p1(point1.x, point1.y, point1.z);
            V3D pl1(point1.lx, point1.ly, point1.lz);
            idxs[m] = point1.id;
            Pose &pose = m_poses[point1.id];
            for (size_t n = 0; n < points.size(); n++)
            {
                const PointType &point2 = points[n];
                V3D p2(point2.x, point2.y, point2.z);
                H_i.block<3, 3>(m * 3, n * 3) = plane_ptr->dp2(p1, p2, m == n);
            }
            J_i.block<1, 3>(0, m * 3) = plane_ptr->dp(p1).transpose();
            D_i.block<3, 3>(m * 3, m * 3) = -pose.r * Sophus::SO3d::hat(pl1);
            D_i.block<3, 3>(m * 3, m * 3 + 3) = M3D::Identity();
        }
        Eigen::MatrixXd H_bar = D_i.transpose() * H_i * D_i; // 6 n * 6 n
        Eigen::VectorXd J_bar = (J_i * D_i).transpose();

        for (size_t m = 0; m < idxs.size(); m++)
        {
            uint32_t p_id1 = idxs[m];
            for (size_t n = m; n < idxs.size(); n++)
            {
                uint32_t p_id2 = idxs[n];
                m_H.block<6, 6>(p_id1 * 6, p_id2 * 6) += H_bar.block<6, 6>(m * 6, n * 6);
                if (m == n)
                    continue;
                m_H.block<6, 6>(p_id2 * 6, p_id1 * 6) += H_bar.block<6, 6>(n * 6, m * 6);
            }
            m_J.block<6, 1>(p_id1 * 6, 0) += J_bar.block<6, 1>(m * 6, 0);
        }
    }
}
void BLAM::optimize()
{
    buildVoxels();
    Eigen::MatrixXd D;
    D.resize(m_poses.size() * 6, m_poses.size() * 6);
    m_H.resize(m_poses.size() * 6, m_poses.size() * 6);
    m_J.resize(m_poses.size() * 6);
    double residual = 0.0;
    bool build_hess = true;
    double u = 0.01, v = 2.0;

    for (size_t i = 0; i < m_config.max_iter; i++)
    {
        if (build_hess)
        {
            residual = updatePlanesByPoses(m_poses);
            updateJaccAndHess();
        }
        D = m_H.diagonal().asDiagonal();
        Eigen::MatrixXd Hess = m_H + u * D;
        Eigen::VectorXd delta = Hess.colPivHouseholderQr().solve(-m_J);
        Vec<Pose> temp_pose(m_poses.begin(), m_poses.end());
        plusDelta(temp_pose, delta);
        double residual_new = evalPlanesByPoses(temp_pose);
        double pho = (residual - residual_new) / (0.5 * (delta.transpose() * (u * D * delta - m_J))[0]);
        if (pho > 0)
        {
            build_hess = true;
            // std::cout << "ITER : " << i << " LM ITER UPDATE" << std::endl;
            m_poses = temp_pose;
            v = 2.0;
            double q = 1 - pow(2 * pho - 1, 3);
            u *= (q < 0.33333333 ? 0.33333333 : q);
        }
        else
        {
            build_hess = false;
            u *= v;
            v *= 2;
        }
        if (std::abs(residual - residual_new) < 1e-9)
            break;
    }
}

double BLAM::updatePlanesByPoses(const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane = m_planes[i];
        residual += plane->updateByPose(poses);
    }
    return residual;
}

double BLAM::evalPlanesByPoses(const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < m_planes.size(); i++)
    {
        OctoTree *plane = m_planes[i];
        residual += plane->evalByPose(poses);
    }
    return residual;
}

int BLAM::planeCount(bool with_sub_planes)
{
    int count = 0;
    for (auto &iter : m_voxel_map)
    {
        if (with_sub_planes)
            count += iter.second->planeCount();
        else if (iter.second->isPlane())
            count += 1;
    }
    return count;
}

void BLAM::plusDelta(Vec<Pose> &poses, const Eigen::VectorXd &x)
{
    assert(static_cast<size_t>(x.rows()) == poses.size() * 6);
    for (size_t i = 0; i < poses.size(); i++)
    {
        M3D &r = poses[i].r;
        V3D &t = poses[i].t;
        r = r * Sophus::SO3d::exp(x.segment<3>(i * 6)).matrix();
        t += x.segment<3>(i * 6 + 3);
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr BLAM::getLocalCloud()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    Pose &first_pose = m_poses[0];

    for (OctoTree *plane : m_planes)
    {
        for (const PointType &point : plane->points())
        {
            pcl::PointXYZI p;
            Pose &cur_pose = m_poses[point.id];
            M3D r_fc = first_pose.r.transpose() * cur_pose.r;
            V3D t_fc = first_pose.r.transpose() * (cur_pose.t - first_pose.t);
            V3D p_vec(point.lx, point.ly, point.lz);
            p_vec = r_fc * p_vec + t_fc;
            p.x = p_vec[0];
            p.y = p_vec[1];
            p.z = p_vec[2];
            p.intensity = point.intensity;
            ret->push_back(p);
        }
    }
    return ret;
}