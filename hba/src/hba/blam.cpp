#include "blam.h"

#include <chrono>
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
void OctoTree::getAllPlanes(Vec<OctoTree *> &planes)
{
    if (!m_is_valid)
        return;
    if (m_is_plane)
    {
        planes.push_back(this);
        return;
    }
    if (m_layer < m_max_layer - 1)
    {
        for (auto &leaf : m_leaves)
        {
            if (leaf != nullptr)
                leaf->getAllPlanes(planes);
        }
    }
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
    if (static_cast<int>(m_points.size()) < m_min_point_num)
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
        m_mean = mean;
        m_eigen_val = eigen_val;
        m_eigen_vec = eigensolver.eigenvectors();
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
M3D OctoTree::Fp(const V3D &p)
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
M3D OctoTree::dpdp(const V3D &p1, const V3D &p2, bool equal)
{
    double n = static_cast<double>(m_points.size());
    V3D u0 = m_eigen_vec.col(0);
    if (equal)
    {
        return 2.0 / n * ((n - 1) / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * Fp(p2) + m_eigen_vec * Fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
    else
    {
        return 2.0 / n * (-1.0 / n * u0 * u0.transpose() + u0 * (p1 - m_mean).transpose() * m_eigen_vec * Fp(p2) + m_eigen_vec * Fp(p2) * (u0.transpose() * (p1 - m_mean)));
    }
}
BLAM::BLAM(const Config &config) : m_config(config)
{
    m_map.clear();
    m_poses.clear();
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
    PointVec points;
    points.reserve(cloud->size());
    for (auto &p : cloud->points)
    {
        PointType point;
        point.lx = p.x;
        point.ly = p.y;
        point.lz = p.z;
        point.intensity = p.intensity;
        point.id = idx;
        point.time = 0.0;
        points.push_back(point);
    }
    m_clouds.push_back(points);
}
void BLAM::buildVoxels()
{
    m_map.clear();
    for (size_t i = 0; i < m_clouds.size(); i++)
    {
        PointVec &points = m_clouds[i];
        Pose &pose = m_poses[i];
        for (auto &p : points)
        {
            V3D p_vec(p.lx, p.ly, p.lz);
            p_vec = pose.r * p_vec + pose.t;
            p.x = p_vec.x();
            p.y = p_vec.y();
            p.z = p_vec.z();
            addPoint(p);
        }
    }

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
void BLAM::getAllPlanes(Vec<OctoTree *> &planes)
{
    for (auto &p : m_map)
    {
        p.second->getAllPlanes(planes);
    }
}
void BLAM::buildMatrix(Vec<OctoTree *> &planes, Vec<PointVec> &all_points, Eigen::MatrixXd &H, Eigen::VectorXd &J)
{
    H.setZero();
    J.setZero();

    for (size_t i = 0; i < planes.size(); i++)
    {
        OctoTree *plane_ptr = planes[i];
        PointVec &points = all_points[i];
        Eigen::MatrixXd D_i = Eigen::MatrixXd::Zero(3 * points.size(), 6 * points.size());
        Eigen::MatrixXd H_i = Eigen::MatrixXd::Zero(3 * points.size(), 3 * points.size());
        Eigen::MatrixXd J_i = Eigen::MatrixXd::Zero(1, 3 * points.size());
        Vec<uint32_t> idxs(points.size(), 0);
        for (size_t m = 0; m < points.size(); m++)
        {
            PointType &point1 = points[m];
            V3D p1(point1.x, point1.y, point1.z);
            V3D pl1(point1.lx, point1.ly, point1.lz);
            idxs[m] = point1.id;
            Pose &pose = m_poses[point1.id];
            for (size_t n = 0; n < points.size(); n++)
            {
                PointType &point2 = points[n];
                V3D p2(point2.x, point2.y, point2.z);
                H_i.block<3, 3>(m * 3, n * 3) = plane_ptr->dpdp(p1, p2, m == n);
            }
            J_i.block<1, 3>(0, m * 3) = plane_ptr->dp(p1).transpose();
            D_i.block<3, 3>(m * 3, m * 3) = -pose.r * Sophus::SO3d::hat(pl1);
            D_i.block<3, 3>(m * 3, m * 3 + 3) = M3D::Identity();
        }

        Eigen::MatrixXd H_bar = D_i.transpose() * H_i * D_i; // 6 n * 6 n
        Eigen::VectorXd J_bar = (J_i * D_i).transpose();     // 6n * 1
        for (size_t m = 0; m < idxs.size(); m++)
        {
            uint32_t p_id1 = idxs[m];
            for (size_t n = m; n < idxs.size(); n++)
            {
                uint32_t p_id2 = idxs[n];
                H.block<6, 6>(p_id1 * 6, p_id2 * 6) += H_bar.block<6, 6>(m * 6, n * 6);
                if (m == n)
                    continue;
                H.block<6, 6>(p_id2 * 6, p_id1 * 6) += H_bar.block<6, 6>(n * 6, m * 6);
            }
            J.block<6, 1>(p_id1 * 6, 0) += J_bar.block<6, 1>(m * 6, 0);
        }
    }
}
double BLAM::evalResidual(const Vec<OctoTree *> &planes, const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < planes.size(); i++)
    {
        OctoTree *plane_ptr = planes[i];
        PointVec &points = plane_ptr->points();
        V3D mean = V3D::Zero();
        M3D cov = M3D::Zero();
        for (size_t m = 0; m < points.size(); m++)
        {
            const Pose &pose = poses[points[m].id];
            V3D pw = pose.r * V3D(points[m].lx, points[m].ly, points[m].lz) + pose.t;
            mean += pw;
            cov += pw * pw.transpose();
        }
        mean /= static_cast<double>(points.size());
        cov = cov / static_cast<double>(points.size()) - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
        residual += eigensolver.eigenvalues()[0];
    }
    return residual;
}
void BLAM::updateMergedPoints(Vec<PointVec> &merged_points, const Vec<Pose> &poses)
{
    for (size_t i = 0; i < merged_points.size(); i++)
    {
        PointVec &points = merged_points[i];
        for (size_t j = 0; j < points.size(); j++)
        {
            PointType &p = points[j];
            const Pose &pose = poses[p.id];
            V3D pw = pose.r * V3D(p.lx, p.ly, p.lz) + pose.t;
            p.x = pw.x();
            p.y = pw.y();
            p.z = pw.z();
        }
    }
}
double BLAM::updatePlanes(const Vec<OctoTree *> &planes, const Vec<Pose> &poses)
{
    double residual = 0.0;
    for (size_t i = 0; i < planes.size(); i++)
    {
        OctoTree *plane_ptr = planes[i];
        PointVec &points = plane_ptr->points();
        V3D mean = V3D::Zero();
        M3D cov = M3D::Zero();
        for (size_t m = 0; m < points.size(); m++)
        {
            const Pose &pose = poses[points[m].id];
            V3D pw = pose.r * V3D(points[m].lx, points[m].ly, points[m].lz) + pose.t;
            mean += pw;
            cov += pw * pw.transpose();
        }
        mean /= static_cast<double>(points.size());
        cov = cov / static_cast<double>(points.size()) - mean * mean.transpose();
        Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
        plane_ptr->m_mean = mean;
        plane_ptr->m_eigen_val = eigensolver.eigenvalues();
        plane_ptr->m_eigen_vec = eigensolver.eigenvectors();
        residual += plane_ptr->m_eigen_val[0];
    }
    return residual;
}
void BLAM::optimize()
{
    buildVoxels();
    Vec<OctoTree *> planes;
    getAllPlanes(planes);
    Vec<PointVec> all_points(planes.size(), PointVec(0));
    Vec<size_t> all_start_idx(planes.size(), 0);

    size_t last_size = 0;
    for (size_t i = 0; i < planes.size(); i++)
    {
        OctoTree *plane = planes[i];
        mergePointVec(plane->points(), all_points[i]);
        all_start_idx[i] = last_size;
        last_size += all_points[i].size();
    }
    Eigen::MatrixXd H, D;
    Eigen::VectorXd J;
    D.resize(m_poses.size() * 6, m_poses.size() * 6);
    H.resize(m_poses.size() * 6, m_poses.size() * 6);
    J.resize(m_poses.size() * 6);

    double residual = 0.0;
    bool build_hess = true;
    double u = 0.01, v = 2.0;

    for (size_t i = 0; i < m_config.max_iter; i++)
    {
        if (build_hess)
        {
            residual = updatePlanes(planes, m_poses);
            updateMergedPoints(all_points, m_poses);
            buildMatrix(planes, all_points, H, J);
            m_H = H;
        }
        D = H.diagonal().asDiagonal();
        Eigen::MatrixXd Hess = H + u * D;
        Eigen::VectorXd delta = Hess.colPivHouseholderQr().solve(-J);
        Vec<Pose> temp_pose(m_poses.begin(), m_poses.end());
        updatePoses(temp_pose, delta);
        double residual_new = evalResidual(planes, temp_pose);
        double pho = (residual - residual_new) / (0.5 * (delta.transpose() * (u * D * delta - J))[0]);
        if (pho > 0)
        {
            build_hess = true;
            // std::cout << "do update" << std::endl;
            // std::cout << m_poses[0].t.transpose() << std::endl;
            // std::cout << delta.segment<6>(0).transpose()<< std::endl;
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
void BLAM::mergePointVec(const PointVec &in, PointVec &out)
{
    std::unordered_map<uint32_t, PointType> temp_cache;
    for (auto &p : in)
    {
        if (temp_cache.find(p.id) == temp_cache.end())
        {
            temp_cache[p.id] = p;
            temp_cache[p.id].time = 1.0;
        }
        else
        {
            temp_cache[p.id].lx += p.lx;
            temp_cache[p.id].ly += p.ly;
            temp_cache[p.id].lz += p.lz;
            temp_cache[p.id].time += 1.0;
        }
    }
    for (auto &iter : temp_cache)
    {
        PointType &p = iter.second;
        p.lx /= p.time;
        p.ly /= p.time;
        p.lz /= p.time;
        out.push_back(p);
    }
}
void BLAM::updatePoses(Vec<Pose> &poses, Eigen::VectorXd &x)
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