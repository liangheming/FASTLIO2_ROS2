#include "image_processor.h"

ImageProcessor::ImageProcessor(Config &config, std::shared_ptr<IESKF> kf)
    : m_kf(kf), m_config(config)
{
    m_frame_count = 0;
    m_camera = std::make_shared<PinholeCamera>(m_config.cam_width, m_config.cam_height,
                                               m_config.cam_fx, m_config.cam_fy,
                                               m_config.cam_cx, m_config.cam_cy,
                                               m_config.cam_d[0], m_config.cam_d[1],
                                               m_config.cam_d[2], m_config.cam_d[3],
                                               m_config.cam_d[4]);
}

void ImageProcessor::process(cv::Mat &img, CloudType::Ptr cloud, bool is_new_cloud)
{
    m_frame_count++;
    if (img.cols != m_camera->width() || img.rows != m_camera->height())
        cv::resize(img, m_cur_img_color, cv::Size2i(m_camera->width(), m_camera->height()));
    else
        m_cur_img_color = img;
    cv::cvtColor(m_cur_img_color, m_cur_img_gray, cv::COLOR_BGR2GRAY);
    m_cur_cloud = cloud;
}

M3D ImageProcessor::r_cw()
{
    const State &s = m_kf->x();
    // return s.r_cl * s.r_il.transpose() * s.r_wi.transpose();
    return m_config.r_cl * s.r_il.transpose() * s.r_wi.transpose();
}

V3D ImageProcessor::t_cw()
{
    const State &s = m_kf->x();
    // return -s.r_cl * s.r_il.transpose() * (s.r_wi.transpose() * s.t_wi + s.t_il) + s.t_cl;
    return -m_config.r_cl * s.r_il.transpose() * (s.r_wi.transpose() * s.t_wi + s.t_il) + m_config.t_cl;
}

M3D ImageProcessor::r_ci()
{
    const State &s = m_kf->x();
    // return s.r_cl * s.r_il.transpose();
    return m_config.r_cl * s.r_il.transpose();
}

V3D ImageProcessor::t_ci()
{
    const State &s = m_kf->x();
    // return -s.r_cl * s.r_il.transpose() * s.t_il + s.t_cl;
    return -m_config.r_cl * s.r_il.transpose() * s.t_il + m_config.t_cl;
}

M3D ImageProcessor::r_wc()
{
    return r_cw().transpose();
}

V3D ImageProcessor::t_wc()
{
    return -r_cw().transpose() * t_cw();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ImageProcessor::getLastestColoredCloud()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ret(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (m_cur_cloud == nullptr || m_cur_cloud->size() <= 0)
        return ret;
    ret->reserve(m_cur_cloud->size());
    for (auto p : m_cur_cloud->points)
    {
        V3D pw = V3D(p.x, p.y, p.z);
        V3D pc = r_cw() * pw + t_cw();

        if (pc(2) <= 0)
            continue;
        V2D px = m_camera->cam2Img(pc);
        if (!m_camera->isInImg(px.cast<int>(), 1))
            continue;
        Eigen::Vector3f bgr = CVUtils::interpolateMat_color(m_cur_img_color, px(0), px(1));
        pcl::PointXYZRGB p_color;
        p_color.x = p.x;
        p_color.y = p.y;
        p_color.z = p.z;
        p_color.b = bgr(0);
        p_color.g = bgr(1);
        p_color.r = bgr(2);
        ret->push_back(p_color);
    }
    return ret;
}