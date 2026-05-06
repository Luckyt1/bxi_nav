#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include <filesystem>

struct GPSPoint {
    double lat;
    double lon;
};

class WaypointFollower : public rclcpp::Node {
public:
    WaypointFollower() : Node("waypoint_follower_node") {
        // 使用 filesystem 获取更可靠的相对路径
        std::filesystem::path current_path = __FILE__;
        std::string default_path = (current_path.parent_path().parent_path() / "data" / "waypoint_3.yaml").string();
        
        this->declare_parameter<std::string>("yaml_path", default_path);
        this->declare_parameter<double>("dist_threshold", 2.0); // 到达阈值，单位：米

         target_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps_waypoint", 10);
        
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/fix", 10, std::bind(&WaypointFollower::gpsCallback, this, std::placeholders::_1));

        loadWaypoints();
    }

private:
    void loadWaypoints() {
        std::string yaml_path = this->get_parameter("yaml_path").as_string();
        try {
            YAML::Node config = YAML::LoadFile(yaml_path);
            if (config["waypoints"]) {
                for (const auto& wp : config["waypoints"]) {
                    waypoints_.push_back({wp["lat"].as<double>(), wp["lon"].as<double>()});
                }
                RCLCPP_INFO(this->get_logger(), "成功加载了 %zu 个 GPS 航点。", waypoints_.size());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML 加载失败: %s", e.what());
        }
    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        if (waypoints_.empty() || current_idx_ >= waypoints_.size()) return;

        double current_lat = msg->latitude;
        double current_lon = msg->longitude;
        
        // 1. 获取当前目标点
        GPSPoint target = waypoints_[current_idx_];

        // 2. 计算当前位置与目标点的距离 (单位: 米)
        double distance = calculateDistance(current_lat, current_lon, target.lat, target.lon);

        // 3. 判定是否到达
        double threshold = this->get_parameter("dist_threshold").as_double();
        if (distance < threshold) {
            RCLCPP_INFO(this->get_logger(), "到达航点 [%zu], 距离: %.2f m。切换下一个...", current_idx_, distance);
            current_idx_++;
        }

        // 4. 持续发布当前目标（供其他节点查看）
        publishTarget();
    }

    // 使用 Haversine 公式计算两个经纬度之间的地面距离
    double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
                   std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
                   std::sin(dLon / 2) * std::sin(dLon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
        return 6371000.0 * c; // 地球平均半径 6371km
    }

    void publishTarget() {
    if (current_idx_ < waypoints_.size()) {
        auto target_msg = sensor_msgs::msg::NavSatFix();
        
        target_msg.header.stamp = this->get_clock()->now();
        target_msg.header.frame_id = "";
        
        target_msg.latitude = waypoints_[current_idx_].lat;
        target_msg.longitude = waypoints_[current_idx_].lon;
        target_msg.altitude = 0.0;
        
        // 设置状态为有效，防止接收方过滤掉
        target_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        
        target_pub_->publish(target_msg);
    }
}

    size_t current_idx_ = 0;
    std::vector<GPSPoint> waypoints_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

};
int main(int argc, char **argv) {

rclcpp::init(argc, argv);

rclcpp::spin(std::make_shared<WaypointFollower>());

rclcpp::shutdown();

return 0;

}