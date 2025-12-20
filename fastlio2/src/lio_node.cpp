#include <mutex>
#include <vector>
#include <queue>
#include <deque>
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstring>   // memcpy
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// ===============================
// Config / State
// ===============================
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "camera_init";
    bool print_time_cost = false;
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

// ===============================
// PointCloud2 helpers
// ===============================
static const sensor_msgs::msg::PointField* find_field(
    const sensor_msgs::msg::PointCloud2& msg,
    const std::string& name)
{
    for (const auto& f : msg.fields) {
        if (f.name == name) return &f;
    }
    return nullptr;
}

template <typename T>
static inline T read_as(const uint8_t* ptr)
{
    T v;
    std::memcpy(&v, ptr, sizeof(T));
    return v;
}

static inline const char* datatype_to_str(uint8_t dt)
{
    using PF = sensor_msgs::msg::PointField;
    switch (dt) {
        case PF::INT8:    return "INT8";
        case PF::UINT8:   return "UINT8";
        case PF::INT16:   return "INT16";
        case PF::UINT16:  return "UINT16";
        case PF::INT32:   return "INT32";
        case PF::UINT32:  return "UINT32";
        case PF::FLOAT32: return "FLOAT32";
        case PF::FLOAT64: return "FLOAT64";
        default:          return "UNKNOWN";
    }
}

// Read field (count==1) as double (covers common datatypes)
static bool read_field_as_double(const sensor_msgs::msg::PointCloud2& /*msg*/,
                                 const sensor_msgs::msg::PointField& f,
                                 const uint8_t* base,
                                 double& out)
{
    using PF = sensor_msgs::msg::PointField;
    if (f.count != 1) return false;

    const uint8_t* p = base + f.offset;

    switch (f.datatype) {
        case PF::FLOAT32: out = static_cast<double>(read_as<float>(p)); return true;
        case PF::FLOAT64: out = read_as<double>(p); return true;
        case PF::UINT8:   out = static_cast<double>(read_as<uint8_t>(p)); return true;
        case PF::INT8:    out = static_cast<double>(read_as<int8_t>(p)); return true;
        case PF::UINT16:  out = static_cast<double>(read_as<uint16_t>(p)); return true;
        case PF::INT16:   out = static_cast<double>(read_as<int16_t>(p)); return true;
        case PF::UINT32:  out = static_cast<double>(read_as<uint32_t>(p)); return true;
        case PF::INT32:   out = static_cast<double>(read_as<int32_t>(p)); return true;
        default: break;
    }
    return false;
}

// Specialized: read timestamp-like field robustly.
// Supports:
//  - count==1: UINT32 / FLOAT32 / FLOAT64 (and other ints)
//  - count==2 & datatype==UINT32: treat as uint64 = low + (high<<32)  (little-endian expected)
static bool read_time_field_as_double(const sensor_msgs::msg::PointCloud2& msg,
                                      const sensor_msgs::msg::PointField& f,
                                      const uint8_t* base,
                                      double& out)
{
    using PF = sensor_msgs::msg::PointField;

    if (f.count == 1) {
        return read_field_as_double(msg, f, base, out);
    }

    if (f.count == 2 && f.datatype == PF::UINT32) {
        const uint8_t* p = base + f.offset;
        uint32_t a = read_as<uint32_t>(p + 0);
        uint32_t b = read_as<uint32_t>(p + 4);

        // sensor_msgs/PointCloud2 is almost always little-endian in practice.
        // If msg.is_bigendian==true we bail out earlier (see lidarCB).
        uint64_t u64 = (static_cast<uint64_t>(b) << 32) | static_cast<uint64_t>(a);
        out = static_cast<double>(u64);
        return true;
    }

    return false;
}

// Infer unit scale for (t_raw - t0) -> milliseconds, based on observed span.
static double infer_time_scale_to_ms(double raw_span)
{
    // Typical per-scan span:
    //  - ns: ~1e8 for 100ms
    //  - us: ~1e5 for 100ms
    //  - ms: ~100 for 100ms
    //  - s:  ~0.1 for 100ms
    if (raw_span > 1e7)  return 1e-6; // ns -> ms
    if (raw_span > 1e4)  return 1e-3; // us -> ms
    if (raw_span > 10.0) return 1.0;  // ms -> ms
    return 1e3;                       // s  -> ms
}

static const char* infer_time_unit_name(double raw_span)
{
    if (raw_span > 1e7)  return "ns";
    if (raw_span > 1e4)  return "us";
    if (raw_span > 10.0) return "ms";
    return "s";
}

// ===============================
// Node
// ===============================
class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");

        // Optional: declare use_sim_time so you can set it from launch/CLI without warnings.
        // (ROS uses this param to decide clock source)
        // this->declare_parameter("use_sim_time", false);

        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, 200,
            std::bind(&LIONode::imuCB, this, std::placeholders::_1));

        m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_node_config.lidar_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 50);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);

        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2],
                                 r_il_vec[3], r_il_vec[4], r_il_vec[5],
                                 r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);

        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order -> clear buffer");
            m_state_data.imu_buffer.clear();
        }

        m_state_data.imu_buffer.emplace_back(
            V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
            V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
            timestamp
        );
        m_state_data.last_imu_time = timestamp;
    }

    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const double frame_time = Utils::getSec(msg->header);

        // ===== One-time dump of fields (VERY important) =====
        RCLCPP_INFO_ONCE(this->get_logger(),
            "PointCloud2 meta: width=%u height=%u point_step=%u row_step=%u is_bigendian=%d fields=%zu",
            msg->width, msg->height, msg->point_step, msg->row_step, msg->is_bigendian, msg->fields.size());

        for (const auto& f : msg->fields) {
            RCLCPP_INFO_ONCE(this->get_logger(),
                " field name=%s offset=%u datatype=%u(%s) count=%u",
                f.name.c_str(), f.offset, f.datatype, datatype_to_str(f.datatype), f.count);
        }

        if (msg->is_bigendian) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "PointCloud2 is big-endian (is_bigendian=1). This parser expects little-endian. Abort.");
            return;
        }

        // ---- Find fields ----
        const auto* fx = find_field(*msg, "x");
        const auto* fy = find_field(*msg, "y");
        const auto* fz = find_field(*msg, "z");
        const auto* fi = find_field(*msg, "intensity");
        const auto* ftag  = find_field(*msg, "tag");
        const auto* fline = find_field(*msg, "line");
        if (!fline) fline = find_field(*msg, "ring");


        // Time field: prefer offset_time (Livox common), fallback timestamp
        const auto* fot = find_field(*msg, "offset_time");
        const auto* fts = find_field(*msg, "timestamp");
        const auto* ft  = (fot ? fot : fts);
        RCLCPP_INFO_ONCE(this->get_logger(),
            "Field presence: intensity=%d tag=%d line/ring=%d offset_time=%d timestamp=%d",
            (fi!=nullptr), (ftag!=nullptr), (fline!=nullptr), (fot!=nullptr), (fts!=nullptr));


        if (!fx || !fy || !fz || !ft) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "PointCloud2 missing required fields (x,y,z and offset_time/timestamp).");
            return;
        }

        RCLCPP_INFO_ONCE(this->get_logger(),
            "Using time field: %s (datatype=%s count=%u)",
            ft->name.c_str(), datatype_to_str(ft->datatype), ft->count);

        const size_t N = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
        if (N == 0 || msg->point_step == 0) return;

        const uint8_t* data = msg->data.data();
        const size_t step = msg->point_step;

        // ===== Pass 1: compute min/max raw time =====
        double t_min = std::numeric_limits<double>::infinity();
        double t_max = -std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < N; ++i) {
            const uint8_t* base = data + i * step;
            double tr = 0.0;
            if (!read_time_field_as_double(*msg, *ft, base, tr)) continue;
            if (!std::isfinite(tr)) continue;
            t_min = std::min(t_min, tr);
            t_max = std::max(t_max, tr);
        }

        if (!std::isfinite(t_min) || !std::isfinite(t_max) || t_max < t_min) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Failed to read valid time field from points. t_min/t_max invalid.");
            return;
        }

        const double raw_span = (t_max - t_min);
        const double scale_to_ms = infer_time_scale_to_ms(raw_span);
        const char* unit_name = infer_time_unit_name(raw_span);

        // ===== Pass 2: build CloudType with curvature=relative_ms =====
        CloudType::Ptr cloud(new CloudType);
        cloud->reserve(N);

        double min_x=1e100, min_y=1e100, min_z=1e100;
        double max_x=-1e100, max_y=-1e100, max_z=-1e100;

        size_t kept = 0, dropped = 0, dropped_big = 0;
        double prev_ms = 0.0;

        const double ABS_MAX = 1e4; // 10km
        size_t tag_zero = 0, line_zero = 0;
        double tag_min = 1e100, tag_max = -1e100;
        double line_min = 1e100, line_max = -1e100;


        for (size_t i = 0; i < N; ++i)
        {
            const uint8_t* base = data + i * step;

            double x=0.0, y=0.0, z=0.0;
            if (!read_field_as_double(*msg, *fx, base, x) ||
                !read_field_as_double(*msg, *fy, base, y) ||
                !read_field_as_double(*msg, *fz, base, z)) {
                dropped++;
                continue;
            }

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                dropped++;
                continue;
            }

            // ---- outlier clip (protect voxelgrid & downstream) ----
            if (std::fabs(x) > ABS_MAX || std::fabs(y) > ABS_MAX || std::fabs(z) > ABS_MAX) {
                dropped_big++;
                continue;
            }

            // ---- intensity ----
            double intensity = 0.0;
            if (fi) {
                (void)read_field_as_double(*msg, *fi, base, intensity);
                if (!std::isfinite(intensity)) intensity = 0.0;
            }

            // ---- time ----
            double tr = 0.0;
            if (!read_time_field_as_double(*msg, *ft, base, tr) || !std::isfinite(tr)) {
                dropped++;
                continue;
            }

            double dt_ms = (tr - t_min) * scale_to_ms;
            if (dt_ms < 0.0) dt_ms = 0.0;
            if (dt_ms < prev_ms) dt_ms = prev_ms;
            prev_ms = dt_ms;

            // ---- update ranges ----
            min_x = std::min(min_x, x); min_y = std::min(min_y, y); min_z = std::min(min_z, z);
            max_x = std::max(max_x, x); max_y = std::max(max_y, y); max_z = std::max(max_z, z);
            
            double tag = 0.0, line = 0.0;
            if (ftag)  (void)read_field_as_double(*msg, *ftag, base, tag);
            if (fline) (void)read_field_as_double(*msg, *fline, base, line);
            
            if (ftag && tag == 0.0) tag = 1.0;
            
            if (ftag && tag >= 128.0) { dropped++; continue; }

            if (ftag) {
                if (tag == 0.0) tag_zero++;
                tag_min = std::min(tag_min, tag);
                tag_max = std::max(tag_max, tag);
            }
            if (fline) {
                if (line == 0.0) line_zero++;
                line_min = std::min(line_min, line);
                line_max = std::max(line_max, line);
            }


            

            PointType p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = static_cast<float>(z);
            p.intensity = static_cast<float>(intensity);
            p.curvature = static_cast<float>(dt_ms);
            
            p.normal_x = static_cast<float>(line); // scan line id
            p.normal_y = static_cast<float>(tag);  // livox tag
            p.normal_z = 0.0f;

            cloud->points.push_back(p);
            kept++;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "XYZ range: x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f], kept=%zu dropped=%zu dropped_big=%zu",
            min_x, max_x, min_y, max_y, min_z, max_z, kept, dropped, dropped_big);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "tag/line: has_tag=%d has_line=%d tag_zero=%zu line_zero=%zu tag[min=%.1f max=%.1f] line[min=%.1f max=%.1f]",
            (ftag!=nullptr), (fline!=nullptr),
            tag_zero, line_zero,
            (ftag?tag_min:0.0), (ftag?tag_max:0.0),
            (fline?line_min:0.0), (fline?line_max:0.0));


        if (cloud->points.empty()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "All points dropped during parsing (kept=0). Check field datatypes/offsets.");
            return;
        }


        const double span_ms = cloud->points.back().curvature - cloud->points.front().curvature;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Lidar parsed OK: N=%zu kept=%zu dropped=%zu raw_span=%.3f(%s) -> scan_span=%.2f ms frame_time=%.6f",
            N, kept, dropped, raw_span, unit_name, span_ms, frame_time);

        // ===== Push to buffer =====
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);

        if (frame_time < m_state_data.last_lidar_time) {
            RCLCPP_WARN(this->get_logger(),
                "Lidar out of order -> clear lidar buffer");
            m_state_data.lidar_buffer.clear();
        }

        m_state_data.lidar_buffer.emplace_back(frame_time, cloud);
        m_state_data.last_lidar_time = frame_time;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;

        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;

            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(),
                      [](PointType &p1, PointType &p2) { return p1.curvature < p2.curvature; });

            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time +
                (m_package.cloud->points.empty() ? 0.0 : (m_package.cloud->points.back().curvature / 1000.0));

            m_state_data.lidar_pushed = true;
        }

        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() &&
               m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }

        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                      CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0) return;
        if (!cloud || cloud->points.empty()) return;


        cloud->height = 1;
        cloud->width  = static_cast<uint32_t>(cloud->points.size());
        cloud->is_dense = false;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub,
                         std::string frame_id, std::string child_frame, const double &time)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;

        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub,
                     std::string frame_id, const double &time)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster,
                     std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped tfm;
        tfm.header.frame_id = frame_id;
        tfm.child_frame_id = child_frame;
        tfm.header.stamp = Utils::getTime(time);

        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        tfm.transform.translation.x = t.x();
        tfm.transform.translation.y = t.y();
        tfm.transform.translation.z = t.z();
        tfm.transform.rotation.x = q.x();
        tfm.transform.rotation.y = q.y();
        tfm.transform.rotation.z = q.z();
        tfm.transform.rotation.w = q.w();
        broad_caster->sendTransform(tfm);
    }

    void timerCB()
    {
        // --- Buffer debug ---
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "buffers: imu=%zu lidar=%zu pushed=%d last_imu=%.3f last_lidar=%.3f",
            m_state_data.imu_buffer.size(),
            m_state_data.lidar_buffer.size(),
            (int)m_state_data.lidar_pushed,
            m_state_data.last_imu_time,
            m_state_data.last_lidar_time);

        if (!syncPackage())
            return;

        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        // Publish odom/path/tf regardless of mapping status
        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);

        // Only publish clouds when mapping (optional)
        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        CloudType::Ptr body_cloud =
            m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        // Use start_time for visualization consistency (your original preference)
        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_start_time);

        CloudType::Ptr world_cloud =
            m_builder->lidar_processor()->transformCloud(m_package.cloud,
                                                        m_builder->lidar_processor()->r_wl(),
                                                        m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_start_time);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
