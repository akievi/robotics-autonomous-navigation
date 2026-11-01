#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanAngleFilterNode : public rclcpp::Node
{
public:
  ScanAngleFilterNode() : Node("scan_angle_filter")
  {
    angle_begin_deg_ = declare_parameter<double>("angle_begin", -80.0);
    angle_end_deg_   = declare_parameter<double>("angle_end", 80.0);

    // QoS compatible with RViz (RELIABLE)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliable()
                 .durability_volatile();
                 
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      qos,
      std::bind(&ScanAngleFilterNode::scan_callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_filtered", qos);

    RCLCPP_INFO(
      get_logger(),
      "Filtering scan from %.2f deg to %.2f deg",
      angle_begin_deg_,
      angle_end_deg_);
  }

private:
  static double deg_to_rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) {
      return;
    }

    sensor_msgs::msg::LaserScan filtered = *msg;

    double angle_begin = deg_to_rad(angle_begin_deg_);
    double angle_end   = deg_to_rad(angle_end_deg_);

    if (angle_begin > angle_end) {
      std::swap(angle_begin, angle_end);
    }

    const float inf = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

      if (angle < angle_begin || angle > angle_end) {
        filtered.ranges[i] = inf;

        if (!filtered.intensities.empty() && i < filtered.intensities.size()) {
          filtered.intensities[i] = 0.0f;
        }
      }
    }

    pub_->publish(filtered);
  }

  double angle_begin_deg_;
  double angle_end_deg_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanAngleFilterNode>());
  rclcpp::shutdown();
  return 0;
}
