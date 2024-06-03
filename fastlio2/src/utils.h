#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

class Utils
{
public:
    // static pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPCL(const sensor_msgs::msg::PointCloud2 &msg);
    // static sensor_msgs::msg::PointCloud2 convertToROS(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    static double getSec(std_msgs::msg::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg, int filter_num, double min_range = 0.5, double max_range = 20.0);
    static builtin_interfaces::msg::Time getTime(const double& sec);
};