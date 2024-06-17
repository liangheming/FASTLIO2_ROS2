#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;

using M6D = Eigen::Matrix<double, 6, 6>;

template <typename T>
using Vec = std::vector<T>;

struct PointXYZIDT
{
    PCL_ADD_POINT4D;
    float lx;
    float ly;
    float lz;
    float intensity;
    uint32_t id;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIDT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, lx, lx)(float, ly, ly)(float, lz, lz)(float, intensity, intensity)(uint32_t, id, id)(double, time, time))
using PointType = PointXYZIDT;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;