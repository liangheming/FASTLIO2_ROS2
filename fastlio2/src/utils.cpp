#include "utils.h"
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    int point_num = msg->point_num;
    cloud->reserve(point_num / filter_num + 1);
    for (int i = 0; i < point_num; i += filter_num)
    {
        if ((msg->points[i].line < 4) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {

            float x = msg->points[i].x;
            float y = msg->points[i].y;
            float z = msg->points[i].z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;
            p.intensity = msg->points[i].reflectivity;
            p.curvature = msg->points[i].offset_time / 1000000.0f;
            cloud->push_back(p);
        }
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
