#include "map_builder.h"
MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
    m_image_processor = std::make_shared<ImageProcessor>(config, kf);
    m_lastest_cloud = nullptr;
    m_is_new_cloud = false;
    m_colored_points_rendered = false;
    m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage &package)
{
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (m_imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return;
    }

    m_imu_processor->undistort(package);

    if (m_status == BuilderStatus::MAP_INIT)
    {
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_lastest_cloud = cloud_world;
        m_is_new_cloud = true;
        m_status = BuilderStatus::MAPPING;
        return;
    }

    if (package.lidar_end)
    {
        m_lidar_processor->process(package);
        m_lastest_cloud = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_is_new_cloud = true;
    }
    else
    {
        if (!m_lastest_cloud)
            return;
        m_image_processor->process(package.image, m_lastest_cloud, m_is_new_cloud);

        if (m_is_new_cloud)
        {
            m_lastest_colored_cloud = m_image_processor->getLastestColoredCloud();
            m_colored_points_rendered = true;
        }
        m_is_new_cloud = false;
    }
}