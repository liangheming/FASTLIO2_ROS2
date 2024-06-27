#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"
#include "image_processor.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    bool &colorRendered() { return m_colored_points_rendered; }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud() { return m_lastest_colored_cloud; }

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }
    std::shared_ptr<LidarProcessor> lidar_processor() { return m_lidar_processor; }

private:
    Config m_config;
    BuilderStatus m_status;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IMUProcessor> m_imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
    std::shared_ptr<ImageProcessor> m_image_processor;
    CloudType::Ptr m_lastest_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_lastest_colored_cloud;
    bool m_colored_points_rendered;
    bool m_is_new_cloud;
};
