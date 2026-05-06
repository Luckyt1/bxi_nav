#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include <string>

// WGS84 椭球参数
static constexpr double WGS84_A  = 6378137.0;
static constexpr double WGS84_B  = 6356752.3142;
static constexpr double WGS84_E2 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);

struct Waypoint {
    double lat, lon, alt, wait_time;
};

class RtkRelatively : public rclcpp::Node {
public:
    RtkRelatively() : Node("rtk_relatively_node") {
        // 1. 参数声明
        this->declare_parameter<std::string>("waypoint_yaml_path", "/home/tang/navigation/zou/autonomous_exploration_development_environment/src/gps_waypoint/data/waypoint_2.yaml");
        this->declare_parameter<double>("arrival_threshold", 3.0);
        
        std::string file_path = this->get_parameter("waypoint_yaml_path").as_string();
        arrival_threshold_ = this->get_parameter("arrival_threshold").as_double();

        // 2. 加载航点
        if (!loadWaypoints(file_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from %s", file_path.c_str());
        }

        // 3. 订阅与发布
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&RtkRelatively::fixCallback, this, std::placeholders::_1));
        
        // 可选：订阅里程计以获取当前朝向（如果需要发布 Pose）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/aft_mapped_to_init", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_odom_ = *msg;
            });

        way_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);

        // 4. 定时器：控制逻辑频率
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&RtkRelatively::controlLoop, this));
    }

private:
    // 将经纬度转换为以 (lat0, lon0) 为原点的 ENU 坐标
    void latLonToENU(double lat, double lon, double alt, double &e, double &n, double &u) {
        double lat_rad = lat0_ * M_PI / 180.0;
        double R_m = WGS84_A * (1.0 - WGS84_E2) / std::pow(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad), 1.5);
        double R_n = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));

        e = (lon - lon0_) * (M_PI / 180.0) * R_n * std::cos(lat_rad);
        n = (lat - lat0_) * (M_PI / 180.0) * R_m;
        u = alt - alt0_;
    }

    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;

        if (!origin_set_) {
            lat0_ = msg->latitude;
            lon0_ = msg->longitude;
            alt0_ = msg->altitude;
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "GPS Origin set: [%f, %f]", lat0_, lon0_);
        }
        curr_lat_ = msg->latitude;
        curr_lon_ = msg->longitude;
        curr_alt_ = msg->altitude;
        has_gps_ = true;
    }

    void controlLoop() {
        if (!origin_set_ || !has_gps_ || waypoints_.empty()) return;
        if (current_idx_ >= waypoints_.size()) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Task Completed.");
            return;
        }

        // 1. 计算当前 ENU 坐标
        double cur_e, cur_n, cur_u;
        latLonToENU(curr_lat_, curr_lon_, curr_alt_, cur_e, cur_n, cur_u);

        // 2. 计算目标 ENU 坐标
        const auto& target = waypoints_[current_idx_];
        double tar_e, tar_n, tar_u;
        latLonToENU(target.lat, target.lon, target.alt, tar_e, tar_n, tar_u);

        // 3. 距离判定
        double dist = std::sqrt(std::pow(tar_e - cur_e, 2) + std::pow(tar_n - cur_n, 2));

        if (dist < arrival_threshold_) {
            if (!is_waiting_) {
                wait_start_time_ = this->now();
                is_waiting_ = true;
                RCLCPP_INFO(this->get_logger(), "Arrived at WP %zu. Waiting %f s...", current_idx_, target.wait_time);
            }

            if ((this->now() - wait_start_time_).seconds() >= target.wait_time) {
                current_idx_++;
                is_waiting_ = false;
                RCLCPP_INFO(this->get_logger(), "Proceeding to next WP.");
            }
            return; 
        }

        // 4. 发布目标点
        geometry_msgs::msg::PointStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map"; // 或者是里程计坐标系
        msg.point.x = tar_e;
        msg.point.y = tar_n;
        msg.point.z = 0.0;
        way_point_pub_->publish(msg);
    }

    bool loadWaypoints(const std::string& path) {
        try {
            YAML::Node node = YAML::LoadFile(path);
            for (const auto& wp : node["waypoints"]) {
                waypoints_.push_back({wp["lat"].as<double>(), wp["lon"].as<double>(), 
                                      wp["alt"].as<double>(), wp["wait_time"].as<double>()});
            }
            return !waypoints_.empty();
        } catch (...) { return false; }
    }
 
    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr way_point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_idx_ = 0;
    bool origin_set_ = false;
    bool has_gps_ = false;
    bool is_waiting_ = false;
    rclcpp::Time wait_start_time_;

    double lat0_, lon0_, alt0_;
    double curr_lat_, curr_lon_, curr_alt_;
    double arrival_threshold_;
    nav_msgs::msg::Odometry current_odom_;
};
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RtkRelatively>());
    rclcpp::shutdown();
    return 0;
}