#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>

using std::placeholders::_1;

class PC2ToLivoxNode : public rclcpp::Node {
public:
  PC2ToLivoxNode() : Node("pc2_to_livox_node") {

    input_topic_  = this->declare_parameter<std::string>(
        "input_topic", "/points_raw");
    output_topic_ = this->declare_parameter<std::string>(
        "output_topic", "/livox/lidar");
    frame_id_ = this->declare_parameter<std::string>(
        "frame_id", "livox");

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PC2ToLivoxNode::callback, this, _1));

    pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        output_topic_, rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
                "PC2 → Livox CustomMsg node started");
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    livox_ros_driver2::msg::CustomMsg out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_;

    out.timebase = msg->header.stamp.nanosec;
    out.point_num = msg->width * msg->height;
    out.lidar_id = 0;
    out.rsvd = {0, 0, 0};


    out.points.reserve(out.point_num);

    // iterators
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    bool has_intensity = false;
    bool has_time = false;

    for (const auto &field : msg->fields) {
      if (field.name == "intensity") has_intensity = true;
      if (field.name == "time" || field.name == "t") has_time = true;
    }

    std::unique_ptr<
      sensor_msgs::PointCloud2ConstIterator<float>> iter_intensity;
    std::unique_ptr<
      sensor_msgs::PointCloud2ConstIterator<float>> iter_time;

    if (has_intensity) {
      iter_intensity.reset(
        new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "intensity"));
    }

    if (has_time) {
      // field name could be "time" or "t"
      try {
        iter_time.reset(
          new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "time"));
      } catch (...) {
        iter_time.reset(
          new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "t"));
      }
    }

    uint32_t idx = 0;
    for (; iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++idx) {

      livox_ros_driver2::msg::CustomPoint p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;

      p.reflectivity = has_intensity ? **iter_intensity : 0.0f;
      p.tag = 0;
      p.line = 0;

      if (has_time) {
        // seconds → nanoseconds
        p.offset_time =
            static_cast<uint32_t>((**iter_time) * 1e9);
        ++(*iter_time);
      } else {
        // fallback: evenly spread within 100ms
        p.offset_time =
            static_cast<uint32_t>(
              (1e8 * idx) / std::max(1u, out.point_num));
      }

      out.points.push_back(p);

      if (has_intensity) ++(*iter_intensity);
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PC2ToLivoxNode>());
  rclcpp::shutdown();
  return 0;
}
