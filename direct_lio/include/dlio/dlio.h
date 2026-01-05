/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

// SYSTEM
#include <atomic>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <thread>

// BOOST
#include <boost/format.hpp>

// PCL
#define PCL_NO_PRECOMPILE

// DLIO
#include <nano_gicp/nano_gicp.h>

namespace dlio
{
enum class SensorType
{
  OUSTER,
  VELODYNE,
  HESAI,
  LIVOX,
  UNKNOWN
};

class OdomNode;
class MapNode;

// Simple configuration structure for DLIO
struct DlioConfig
{
  // Version
  std::string version = "0.0.0";

  // Frames
  std::string odom_frame = "odom";
  std::string baselink_frame = "base_link";
  std::string lidar_frame = "lidar";
  std::string imu_frame = "imu";

  // Point cloud processing
  bool deskew_ = true;
  bool vf_use_ = true;
  double vf_res_ = 0.05;
  double crop_size_ = 1.0;

  // Odometry parameters
  double gravity_ = 9.80665;
  bool time_offset_ = false;
  double keyframe_thresh_dist_ = 0.1;
  double keyframe_thresh_rot_ = 1.0;

  // Submap parameters
  int submap_knn_ = 10;
  int submap_kcv_ = 10;
  int submap_kcc_ = 10;

  // Map parameters
  bool densemap_filtered_ = true;
  bool wait_until_move_ = false;

  // Sparse map parameters
  bool publish_sparse_map_ = true;
  double sparse_map_leaf_size_ = 0.25;

  // Adaptive parameters
  bool adaptive_params_ = true;

  // IMU parameters
  bool calibrate_accel_ = true;
  bool calibrate_gyro_ = true;
  double imu_calib_time_ = 3.0;
  int imu_buffer_size_ = 2000;
  bool gravity_align_ = true;
  bool imu_calibrate_ = true;

  // Extrinsics
  std::vector<double> baselink2imu_t = { 0., 0., 0. };
  std::vector<double> baselink2imu_R = { 1., 0., 0., 0., 1., 0., 0., 0., 1. };
  std::vector<double> baselink2lidar_t = { 0., 0., 0. };
  std::vector<double> baselink2lidar_R = { 1., 0., 0., 0., 1., 0., 0., 0., 1. };

  // IMU bias
  std::vector<double> accel_bias = { 0., 0., 0. };
  std::vector<double> gyro_bias = { 0., 0., 0. };
  std::vector<double> accel_sm = { 1., 0., 0., 0., 1., 0., 0., 0., 1. };
};

struct Point
{
  Point() : data{ 0.f, 0.f, 0.f, 1.f }
  {
  }

  PCL_ADD_POINT4D;
  float intensity;  // intensity
  union
  {
    std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
    float time;        // (Velodyne) time since beginning of scan in seconds
    double timestamp;  // (Hesai) absolute timestamp in seconds
                       // (Robosense) absolute timestamp in seconds
                       // (Livox) absolute timestamp in (seconds * 10e9)
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace dlio

POINT_CLOUD_REGISTER_POINT_STRUCT(dlio::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      std::uint32_t, t, t)(float, time, time)(double, timestamp, timestamp))

typedef dlio::Point PointType;
