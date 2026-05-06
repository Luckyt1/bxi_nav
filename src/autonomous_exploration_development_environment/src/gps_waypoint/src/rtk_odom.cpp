/**
 * rtk_odom.cpp
 * 融合 RTK GPS 位置 + 里程计朝向，发布统一里程计到 /state_estimation
 *
 * 位置来源：/fix (sensor_msgs/NavSatFix) → 转为本地 ENU 坐标 (X=东, Y=北, Z=上)
 * 朝向来源：/aft_mapped_to_init (nav_msgs/Odometry) → 直接使用其四元数
 * 第一个有效 GPS 点作为本地坐标原点 (0, 0, 0)
 */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <cmath>
#include <string>

// WGS84 椭球参数
static constexpr double WGS84_A  = 6378137.0;
static constexpr double WGS84_B  = 6356752.3142;
static constexpr double WGS84_E2 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);

class RtkOdometry : public rclcpp::Node
{
public:
  RtkOdometry() : Node("rtk_odometry_node")
  {
    this->declare_parameter<std::string>("fix_topic",        "/fix");
    this->declare_parameter<std::string>("src_odom_topic",   "/aft_mapped_to_init");
    this->declare_parameter<std::string>("odom_topic",       "/state_estimation_t");
    this->declare_parameter<std::string>("frame_id",         "gps_map");
    this->declare_parameter<std::string>("child_frame_id",   "gps_base");
    this->declare_parameter<bool>("publish_tf",             true);

    const auto fix_topic      = this->get_parameter("fix_topic").as_string();
    const auto src_odom_topic = this->get_parameter("src_odom_topic").as_string();
    const auto odom_topic     = this->get_parameter("odom_topic").as_string();

    odom_pub_       = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 订阅 GPS 位置
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic, 10,
      std::bind(&RtkOdometry::fixCallback, this, std::placeholders::_1));

    // 订阅源里程计，仅提取朝向
    src_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      src_odom_topic, 10,
      std::bind(&RtkOdometry::srcOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "RTK Odometry 节点已启动\n  GPS 位置: %s\n  朝向来源: %s\n  发布到:   %s",
      fix_topic.c_str(), src_odom_topic.c_str(), odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "");
  }
private:
  // ---------- 源里程计回调：仅缓存朝向 ----------
  void srcOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_orientation_   = msg->pose.pose.orientation;
    orientation_received_ = true;
  }
  // ---------- GPS 回调：转换位置并融合朝向后发布 ----------
  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    // 只接受有效定位（STATUS_FIX 及以上）
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "GPS 尚未定位 (status=%d)，跳过...", msg->status.status);
      return;
    }
    // 用第一个有效点作为本地坐标原点
    if (!origin_set_) {
      lat0_       = msg->latitude;
      lon0_       = msg->longitude;
      alt0_       = msg->altitude;
      origin_set_ = true;
      RCLCPP_INFO(this->get_logger(),
        "原点已设定: lat=%.8f°, lon=%.8f°, alt=%.3f m",
        lat0_, lon0_, alt0_);
      return;
    }

    if (!orientation_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "尚未收到源里程计朝向，使用单位四元数...");
    }

    // 将 (lat, lon, alt) 转换到 ENU 坐标 (east, north, up)
    double east, north, up;
    latLonToENU(msg->latitude, msg->longitude, msg->altitude, east, north, up);
    // 填充融合后的 Odometry 消息
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = msg->header.stamp;
    odom.header.frame_id = this->get_parameter("frame_id").as_string();
    odom.child_frame_id  = this->get_parameter("child_frame_id").as_string();
    // 位置：来自 GPS ENU 转换
    odom.pose.pose.position.x = east;
    odom.pose.pose.position.y = north;
    odom.pose.pose.position.z = up;
    // 朝向：来自源里程计（有则使用，无则单位四元数）
    if (orientation_received_) {
      odom.pose.pose.orientation = latest_orientation_;
    //    RCLCPP_INFO(this->get_logger(),
    //   "四元素接收值: x=%.4f, y=%.4f, z=%.4f, w=%.4f",
    //   latest_orientation_.x, latest_orientation_.y, latest_orientation_.z, latest_orientation_.w);
    }
    // 位置协方差（若 NavSatFix 提供则使用，否则给默认值 1 m²）
    odom_pub_->publish(odom);

    // 广播 TF：gps_map -> gps_base，与 Point-LIO 的 camera_init->aft_mapped 互不干扰
    if (this->get_parameter("publish_tf").as_bool()) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp    = odom.header.stamp;
      tf_msg.header.frame_id = odom.header.frame_id;
      tf_msg.child_frame_id  = odom.child_frame_id;
      tf_msg.transform.translation.x = east;
      tf_msg.transform.translation.y = north;
      tf_msg.transform.translation.z = up;
      tf_msg.transform.rotation      = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  // ---------- 坐标转换（基于 WGS84 局部线性化）----------
  // 结果为以 (lat0_, lon0_, alt0_) 为原点的 ENU 坐标（单位：米）
  void latLonToENU(double lat, double lon, double alt,
                   double & east, double & north, double & up)
  {
    const double lat_rad = lat0_ * M_PI / 180.0;

    // 子午圈曲率半径 R_m（南北方向）
    const double R_m = WGS84_A * (1.0 - WGS84_E2) /
      std::pow(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad), 1.5);

    // 卯酉圈曲率半径 R_n（东西方向）
    const double R_n = WGS84_A /
      std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));

    east  = (lon - lon0_) * (M_PI / 180.0) * R_n * std::cos(lat_rad);
    north = (lat - lat0_) * (M_PI / 180.0) * R_m;
    up    = alt - alt0_;
  }

  // ---------- 成员变量 ----------
  bool   origin_set_          = false;
  bool   orientation_received_ = false;
  double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;

  geometry_msgs::msg::Quaternion latest_orientation_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     src_odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RtkOdometry>());
  rclcpp::shutdown();
  return 0;
}
