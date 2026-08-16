#pragma once
#include "commons.h"
#include "ieskf.h"
#include "pinhole_camera.h"

class ImageProcessor
{
public:
    ImageProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void process(cv::Mat &img, CloudType::Ptr cloud, bool is_new_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLastestColoredCloud();


    M3D r_cw();
    V3D t_cw();
    M3D r_ci();
    V3D t_ci();
    M3D r_wc();
    V3D t_wc();

private:
    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<PinholeCamera> m_camera;
    cv::Mat m_cur_img_color;
    cv::Mat m_cur_img_gray;
    CloudType::Ptr m_cur_cloud;
    u_int64_t m_frame_count;
};