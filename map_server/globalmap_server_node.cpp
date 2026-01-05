#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GlobalmapServer : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  explicit GlobalmapServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("globalmap_server", options) {
    // declare parameters with defaults
    this->declare_parameter<std::string>("globalmap_pcd", "");
    this->declare_parameter<double>("downsample_resolution", 0.1);
    this->declare_parameter<bool>("convert_utm_to_local", false);
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("globalmap_topic", "/globalmap");

    // read parameters
    globalmap_pcd_ = this->get_parameter("globalmap_pcd").as_string();
    downsample_resolution_ =
        this->get_parameter("downsample_resolution").as_double();
    convert_utm_to_local_ =
        this->get_parameter("convert_utm_to_local").as_bool();
    frame_id_ = this->get_parameter("frame_id").as_string();
    globalmap_topic_ = this->get_parameter("globalmap_topic").as_string();

    // print parameters at startup
    RCLCPP_INFO(this->get_logger(), "Parameters:");
    RCLCPP_INFO(this->get_logger(), "  globalmap_pcd: %s",
                globalmap_pcd_.c_str());
    RCLCPP_INFO(this->get_logger(), "  downsample_resolution: %f",
                downsample_resolution_);
    RCLCPP_INFO(this->get_logger(), "  convert_utm_to_local: %s",
                convert_utm_to_local_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  globalmap_topic: %s",
                globalmap_topic_.c_str());

    // transient_local QoS for latched-like behavior
    rclcpp::QoS qos(10);
    qos.transient_local();
    globalmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        globalmap_topic_, qos);

    map_update_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/map_request/pcd", 10,
        std::bind(&GlobalmapServer::map_update_callback, this,
                  std::placeholders::_1));

    // If a map was provided via parameter, try loading it now
    if (!globalmap_pcd_.empty()) {
      if (!load_globalmap(globalmap_pcd_)) {
        RCLCPP_WARN(this->get_logger(), "Failed to load initial global map: %s",
                    globalmap_pcd_.c_str());
      } else {
        // publish once (latched)
        publish_globalmap();
      }
    }

    // periodic timer: keep publishing the map (helps late-joining RViz clients)
    globalmap_pub_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GlobalmapServer::publish_globalmap, this));

    RCLCPP_INFO(this->get_logger(), "GlobalmapServer initialized");
  }

  ~GlobalmapServer() override = default;

private:
  bool load_globalmap(const std::string &pcd_path) {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());

    if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load globalmap PCD file: %s",
                   pcd_path.c_str());
      return false;
    }

    cloud->header.frame_id = frame_id_;

    // Optionally apply UTM offset if a .utm file exists and the param is set
    if (convert_utm_to_local_) {
      std::ifstream utm_file(pcd_path + ".utm");
      if (utm_file.is_open()) {
        double utm_easting = 0.0, utm_northing = 0.0, altitude = 0.0;
        utm_file >> utm_easting >> utm_northing >> altitude;
        for (auto &pt : cloud->points) {
          pt.x -= static_cast<float>(utm_easting);
          pt.y -= static_cast<float>(utm_northing);
          pt.z -= static_cast<float>(altitude);
        }
        RCLCPP_INFO(this->get_logger(),
                    "Applied UTM offset: easting=%f northing=%f altitude=%f",
                    utm_easting, utm_northing, altitude);
      }
    }

    // Downsample if requested
    if (downsample_resolution_ > 0.0) {
      pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
      pcl::VoxelGrid<PointT> voxelgrid;
      voxelgrid.setLeafSize(downsample_resolution_, downsample_resolution_,
                            downsample_resolution_);
      voxelgrid.setInputCloud(cloud);
      voxelgrid.filter(*filtered);
      cloud = filtered;
    }

    // store the cloud with thread-safety
    {
      std::lock_guard<std::mutex> lk(globalmap_mutex_);
      globalmap_ = cloud;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded global map: %s (points=%zu)",
                pcd_path.c_str(), cloud->size());
    return true;
  }

  void publish_globalmap() {
    std::lock_guard<std::mutex> lk(globalmap_mutex_);
    if (!globalmap_) {
      RCLCPP_WARN(this->get_logger(), "No global map to publish");
      return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*globalmap_, msg);
    msg.header.frame_id = frame_id_;
    msg.header.stamp = this->now();

    globalmap_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published global map (points=%zu)",
                globalmap_->size());
  }

  void publish_once_cb() {
    // kept for backward-compatibility; now we publish repeatedly elsewhere
    publish_globalmap();
  }

  void map_update_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (!msg)
      return;
    RCLCPP_INFO(this->get_logger(), "Received map update request: %s",
                msg->data.c_str());
    if (load_globalmap(msg->data)) {
      publish_globalmap();
    }
  }

private:
  // parameters
  std::string globalmap_pcd_;
  double downsample_resolution_ = 0.1;
  bool convert_utm_to_local_ = false;
  std::string frame_id_ = "map";
  std::string globalmap_topic_ = "/globalmap";

  // ROS interfaces
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub_;
  rclcpp::TimerBase::SharedPtr globalmap_pub_timer_;

  // data
  pcl::PointCloud<PointT>::Ptr globalmap_;
  std::mutex globalmap_mutex_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();

  auto node = std::make_shared<GlobalmapServer>(options);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}