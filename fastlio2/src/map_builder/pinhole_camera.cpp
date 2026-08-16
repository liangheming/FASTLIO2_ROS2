#include "pinhole_camera.h"

PinholeCamera::PinholeCamera(double width, double height,
                             double fx, double fy,
                             double cx, double cy,
                             double d0, double d1, double d2, double d3, double d4)
    : m_width(width), m_height(height),
      m_fx(fx), m_fy(fy), m_cx(cx), m_cy(cy),
      m_distort(fabs(d0) > 0.0000001)
{
    m_d[0] = d0;
    m_d[1] = d1;
    m_d[2] = d2;
    m_d[3] = d3;
    m_d[4] = d4;
    m_cvK = (cv::Mat_<float>(3, 3) << m_fx, 0.0, m_cx, 0.0, m_fy, m_cy, 0.0, 0.0, 1.0);
    m_cvD = (cv::Mat_<float>(1, 5) << m_d[0], m_d[1], m_d[2], m_d[3], m_d[4]);
}

V3D PinholeCamera::img2Cam(const double &u, const double &v) const
{
    V3D xyz;
    if (!m_distort)
    {
        xyz[0] = (u - m_cx) / m_fx;
        xyz[1] = (v - m_cy) / m_fy;
        xyz[2] = 1.0;
    }
    else
    {
        cv::Point2f uv(u, v), px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, m_cvK, m_cvD);
        xyz[0] = px.x;
        xyz[1] = px.y;
        xyz[2] = 1.0;
    }
    return xyz.normalized();
}

V3D PinholeCamera::img2Cam(const V2D &uv) const
{
    return img2Cam(uv(0), uv(1));
}

V2D PinholeCamera::cam2Img(const V2D &uv) const
{
    V2D px;
    if (!m_distort)
    {
        px[0] = m_fx * uv[0] + m_cx;
        px[1] = m_fy * uv[1] + m_cy;
    }
    else
    {
        double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
        x = uv[0];
        y = uv[1];
        r2 = x * x + y * y;
        r4 = r2 * r2;
        r6 = r4 * r2;
        a1 = 2 * x * y;
        a2 = r2 + 2 * x * x;
        a3 = r2 + 2 * y * y;
        cdist = 1 + m_d[0] * r2 + m_d[1] * r4 + m_d[4] * r6;
        xd = x * cdist + m_d[2] * a1 + m_d[3] * a2;
        yd = y * cdist + m_d[2] * a3 + m_d[3] * a1;
        px[0] = xd * m_fx + m_cx;
        px[1] = yd * m_fy + m_cy;
    }
    return px;
}

V2D PinholeCamera::cam2Img(const V3D &xyz) const
{
    return cam2Img(project2d(xyz));
}

bool PinholeCamera::isInImg(const Eigen::Vector2i &obs, int boundary) const
{
    if (obs[0] >= boundary && obs[0] < width() - boundary && obs[1] >= boundary && obs[1] < height() - boundary)
        return true;
    return false;
}

Eigen::Matrix<double, 2, 3> PinholeCamera::dpi(V3D pc)
{
    Eigen::Matrix<double, 2, 3> J;
    const double x = pc[0];
    const double y = pc[1];
    const double z_inv = 1. / pc[2];
    const double z_inv_2 = z_inv * z_inv;
    J(0, 0) = m_fx * z_inv;
    J(0, 1) = 0.0;
    J(0, 2) = -m_fx * x * z_inv_2;
    J(1, 0) = 0.0;
    J(1, 1) = m_fy * z_inv;
    J(1, 2) = -m_fy * y * z_inv_2;
    return J;
}

float CVUtils::interpolateMat_8u(const cv::Mat &mat, float u, float v)
{
    assert(mat.type() == CV_8U);
    int x = floor(u);
    int y = floor(v);
    float subpix_x = u - x;
    float subpix_y = v - y;

    float wtl = (1.0f - subpix_x) * (1.0f - subpix_y);
    float wtr = subpix_x * (1.0f - subpix_y);
    float wbl = (1.0f - subpix_x) * subpix_y;
    float wbr = subpix_x * subpix_y;
    float ptl = static_cast<float>(mat.ptr<uchar>(y)[x]);
    float ptr = static_cast<float>(mat.ptr<uchar>(y)[x + 1]);
    float pbl = static_cast<float>(mat.ptr<uchar>(y + 1)[x]);
    float pbr = static_cast<float>(mat.ptr<uchar>(y + 1)[x + 1]);

    return wtl * ptl + wtr * ptr + wbl * pbl + wbr * pbr;
}

float CVUtils::shiTomasiScore(const cv::Mat &img, int u, int v)
{
    assert(img.type() == CV_8UC1);

    float dXX = 0.0;
    float dYY = 0.0;
    float dXY = 0.0;
    const int halfbox_size = 4;
    const int box_size = 2 * halfbox_size;
    const int box_area = box_size * box_size;
    const int x_min = u - halfbox_size;
    const int x_max = u + halfbox_size;
    const int y_min = v - halfbox_size;
    const int y_max = v + halfbox_size;

    if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 || y_max >= img.rows - 1)
        return 0.0; // patch is too close to the boundary

    const int stride = img.step.p[0];
    for (int y = y_min; y < y_max; ++y)
    {
        const uint8_t *ptr_left = img.data + stride * y + x_min - 1;
        const uint8_t *ptr_right = img.data + stride * y + x_min + 1;
        const uint8_t *ptr_top = img.data + stride * (y - 1) + x_min;
        const uint8_t *ptr_bottom = img.data + stride * (y + 1) + x_min;
        for (int x = 0; x < box_size; ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom)
        {
            float dx = *ptr_right - *ptr_left;
            float dy = *ptr_bottom - *ptr_top;
            dXX += dx * dx;
            dYY += dy * dy;
            dXY += dx * dy;
        }
    }

    // Find and return smaller eigenvalue:
    dXX = dXX / (2.0 * box_area);
    dYY = dYY / (2.0 * box_area);
    dXY = dXY / (2.0 * box_area);
    return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
}

Eigen::Vector3f CVUtils::interpolateMat_color(const cv::Mat &mat, float u, float v)
{
    assert(mat.type() == CV_8UC3);

    int x = floor(u);
    int y = floor(v);
    float subpix_x = u - x;
    float subpix_y = v - y;
    float wtl = (1.0f - subpix_x) * (1.0f - subpix_y);
    float wtr = subpix_x * (1.0f - subpix_y);
    float wbl = (1.0f - subpix_x) * subpix_y;
    float wbr = subpix_x * subpix_y;

    cv::Vec3b ptl = mat.ptr<cv::Vec3b>(y)[x];
    cv::Vec3b ptr = mat.ptr<cv::Vec3b>(y)[x + 1];
    cv::Vec3b pbl = mat.ptr<cv::Vec3b>(y + 1)[x];
    cv::Vec3b pbr = mat.ptr<cv::Vec3b>(y + 1)[x + 1];

    Eigen::Vector3f vtl(ptl[0], ptl[1], ptl[2]);
    Eigen::Vector3f vtr(ptr[0], ptr[1], ptr[2]);
    Eigen::Vector3f vbl(pbl[0], pbl[1], pbl[2]);
    Eigen::Vector3f vbr(pbr[0], pbr[1], pbr[2]);

    return wtl * vtl + wtr * vtr + wbl * vbl + wbr * vbr;
}

bool CVUtils::getPatch(cv::Mat img, const V2D px, cv::Mat &patch, int half_patch, int level)
{

    int height = img.rows;
    int width = img.cols;
    assert(img.type() == CV_8U);
    assert(patch.type() == CV_32F);
    assert(patch.cols == 2 * half_patch + 1 && patch.rows == 2 * half_patch + 1);
    const float u_ref = px[0];
    const float v_ref = px[1];
    const int scale = (1 << level);
    const int u_ref_i = floorf(px[0] / scale) * scale;
    const int v_ref_i = floorf(px[1] / scale) * scale;
    const float subpix_u_ref = (u_ref - u_ref_i) / scale;
    const float subpix_v_ref = (v_ref - v_ref_i) / scale;

    const float w_tl = (1.0 - subpix_u_ref) * (1.0 - subpix_v_ref);
    const float w_tr = subpix_u_ref * (1.0 - subpix_v_ref);
    const float w_bl = (1.0 - subpix_u_ref) * subpix_v_ref;
    const float w_br = subpix_u_ref * subpix_v_ref;
    if (u_ref_i - half_patch < 0 || u_ref_i + half_patch >= width || v_ref_i - half_patch < 0 || v_ref_i + half_patch >= height)
        return false;

    for (int y = 0; y <= half_patch * 2; y++)
    {
        for (int x = 0; x <= half_patch * 2; x++)
        {
            V2D px_patch(x - half_patch, y - half_patch);
            px_patch *= (1 << level);
            int tl_x = u_ref_i + px_patch(0), tl_y = v_ref_i + px_patch(1);
            uint8_t tl = img.ptr<uint8_t>(tl_y)[tl_x];
            uint8_t tr = img.ptr<uint8_t>(tl_y)[tl_x + 1];
            uint8_t bl = img.ptr<uint8_t>(tl_y + 1)[tl_x];
            uint8_t br = img.ptr<uint8_t>(tl_y + 1)[tl_x + 1];
            patch.ptr<float>(y)[x] = w_tl * tl + w_tr * tr + w_bl * bl + w_br * br;
        }
    }
    return true;
}

float CVUtils::weightPixel(cv::Mat img, Eigen::Vector4f &weight, Eigen::Vector2i &tl_xy)
{
    assert(img.type() == CV_8U);
    int tl_x = tl_xy(0), tl_y = tl_xy(1);
    float w_tl = weight(0), w_tr = weight(1), w_bl = weight(2), w_br = weight(3);
    float tl = static_cast<float>(img.ptr<uint8_t>(tl_y)[tl_x]);
    float tr = static_cast<float>(img.ptr<uint8_t>(tl_y)[tl_x + 1]);
    float bl = static_cast<float>(img.ptr<uint8_t>(tl_y + 1)[tl_x]);
    float br = static_cast<float>(img.ptr<uint8_t>(tl_y + 1)[tl_x + 1]);
    return w_tl * tl + w_tr * tr + w_bl * bl + w_br * br;
}