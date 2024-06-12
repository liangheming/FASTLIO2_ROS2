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
void BLAM::buildMatrix(Vec<OctoTree *> &planes, Vec<PointVec> &all_points, Vec<size_t> all_start_idx, Eigen::MatrixXd &H, Eigen::MatrixXd &D, Eigen::MatrixXd &J)
{
    H.setZero();
    D.setZero();
    J.setZero();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (size_t i = 0; i < planes.size(); i++)
    {
        OctoTree *plane_ptr = planes[i];
        size_t start_idx = all_start_idx[i];
        PointVec &points = all_points[i];

        for (size_t m = 0; m < points.size(); m++)
        {
            V3D p1 = V3D(points[m].x, points[m].y, points[m].z);
            V3D pl1 = V3D(points[m].lx, points[m].ly, points[m].lz);
            size_t row = (start_idx + m) * 3;
            uint32_t d_col = points[m].id * 6;
            Pose &pose = m_poses[points[m].id];
            for (size_t n = 0; n < points.size(); n++)
            {
                V3D p2 = V3D(points[n].x, points[n].y, points[n].z);
                // V3D pl2 = V3D(points[n].lx, points[n].ly, points[n].lz);
                size_t col = (start_idx + n) * 3;
                H.block<3, 3>(row, col) = plane_ptr->dpdp(p1, p2, m == n);
            }
            J.block<1, 3>(0, row) = plane_ptr->dp(p1).transpose();
            M3D skew_pl;
            skew_pl << SKEW_SYM_MATRX(pl1);
            D.block<3, 3>(row, d_col) = -pose.r * skew_pl;
            D.block<3, 3>(row, d_col + 3) = M3D::Identity();
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
    // auto t0 = std::chrono::high_resolution_clock::now();
    buildVoxels();
    // auto t1 = std::chrono::high_resolution_clock::now();

    // std::chrono::duration<double> duration = t1 - t0;
    // std::cout << "buildVoxels: " << duration.count() << " s" << std::endl;

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

    Eigen::MatrixXd H, D, J, I;
    H.resize(last_size * 3, last_size * 3);
    D.resize(last_size * 3, m_poses.size() * 6);
    J.resize(1, last_size * 3);
    I.resize(m_poses.size() * 6, m_poses.size() * 6);
    I.setIdentity();

    double u = 0.01, v = 2.0, residual = 0.0, q = 0.0;
    bool calc_hess = true;

    for (size_t i = 0; i < m_config.max_iter; i++)
    {
        if (calc_hess)
        {
            updateMergedPoints(all_points, m_poses);

            residual = updatePlanes(planes, m_poses);
            // t0 = std::chrono::high_resolution_clock::now();
            buildMatrix(planes, all_points, all_start_idx, H, D, J);
            // t1 = std::chrono::high_resolution_clock::now();
            // duration = t1 - t0;
            // std::cout << "buildMatrix: " << duration.count() << " s" << std::endl;
        }
        // t0 = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd H_telta = D.transpose() * H * D + u * I;
        // t1 = std::chrono::high_resolution_clock::now();
        // duration = t1 - t0;
        // std::cout << "H_telta: " << duration.count() << " s" << std::endl;

        Eigen::VectorXd J_telta = (J * D).transpose();

        // t0 = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd b_telta = H_telta.colPivHouseholderQr().solve(-J_telta);
        // t1 = std::chrono::high_resolution_clock::now();
        // duration = t1 - t0;
        // std::cout << "solve: " << duration.count() << " s" << std::endl;

        Vec<Pose> temp_pose(m_poses.begin(), m_poses.end());
        updatePoses(temp_pose, b_telta);
        double residual2 = evalResidual(planes, temp_pose);
        q = residual - residual2;
        if (q > 0)
        {
            m_poses = temp_pose;
            double q1 = 0.5 * (b_telta.transpose() * (u * H_telta.diagonal().asDiagonal() * b_telta - J_telta))[0];
            q = q / q1;
            v = 2.0;
            q = 1 - pow(2 * q - 1, 3);
            u *= (q < 0.33333333333 ? 0.33333333333 : q);
            calc_hess = true;
            break;
        }
        else
        {
            u = u * v;
            v = 2 * v;
            calc_hess = false;
        }
        if (std::abs(residual2 - residual) < 1e-6)
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
            // temp_cache[p.id].x += p.x;
            // temp_cache[p.id].y += p.y;
            // temp_cache[p.id].z += p.z;
            temp_cache[p.id].lx += p.lx;
            temp_cache[p.id].ly += p.ly;
            temp_cache[p.id].lz += p.lz;
            temp_cache[p.id].time += 1.0;
        }
    }
    for (auto &iter : temp_cache)
    {
        PointType &p = iter.second;
        // p.x /= p.time;
        // p.y /= p.time;
        // p.z /= p.time;
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