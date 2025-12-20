#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class LivoxCustomToPC2 : public rclcpp::Node
{
public:
    LivoxCustomToPC2() : Node("livox_custom_to_pc2")
    {
        this->declare_parameter<std::string>("input_topic", "/livox/lidar");
        this->declare_parameter<std::string>("output_topic", "/livox/lidar_pc2");
        this->declare_parameter<std::string>("frame_id", "lidar");

        input_topic_  = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        frame_id_     = this->get_parameter("frame_id").as_string();

        sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            input_topic_, 10,
            std::bind(&LivoxCustomToPC2::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, 10);

        RCLCPP_INFO(this->get_logger(),
            "Livox Custom → PointCloud2 started\n  in:  %s\n  out: %s",
            input_topic_.c_str(), output_topic_.c_str());
    }

private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.reserve(msg->point_num);

        for (uint32_t i = 0; i < msg->point_num; ++i)
        {
            const auto &p = msg->points[i];
            pcl::PointXYZI pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            pt.intensity = static_cast<float>(p.reflectivity);
            cloud.push_back(pt);
        }

        sensor_msgs::msg::PointCloud2 pc2;
        pcl::toROSMsg(cloud, pc2);
        pc2.header.stamp = msg->header.stamp;
        pc2.header.frame_id = frame_id_;

        pub_->publish(pc2);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxCustomToPC2>());
    rclcpp::shutdown();
    return 0;
}
