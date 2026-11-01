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
    angle_begin_deg_ = declare_parameter<double>("angle_begin", -10.0);
    angle_end_deg_   = declare_parameter<double>("angle_end",   10.0);

    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "angle_begin")
            angle_begin_deg_ = p.as_double();
          else if (p.get_name() == "angle_end")
            angle_end_deg_ = p.as_double();
        }
        RCLCPP_INFO(get_logger(), "Lidar window updated: %.1f° to %.1f°",
                    angle_begin_deg_, angle_end_deg_);
        return result;
      });

    // RPLidar driver publishes /scan as BEST_EFFORT — must match here
    auto sub_qos = rclcpp::SensorDataQoS();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sub_qos,
      std::bind(&ScanAngleFilterNode::scan_callback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", pub_qos);

    RCLCPP_INFO(get_logger(), "Filtering scan from %.2f deg to %.2f deg",
                angle_begin_deg_, angle_end_deg_);
  }

private:
  static double deg_to_rad(double deg) { return deg * M_PI / 180.0; }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (msg->ranges.empty()) return;

    sensor_msgs::msg::LaserScan filtered = *msg;

    double angle_begin = deg_to_rad(angle_begin_deg_);
    double angle_end   = deg_to_rad(angle_end_deg_);

    if (angle_begin > angle_end) std::swap(angle_begin, angle_end);

    const float inf = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (angle < angle_begin || angle > angle_end) {
        filtered.ranges[i] = inf;
        if (!filtered.intensities.empty() && i < filtered.intensities.size())
          filtered.intensities[i] = 0.0f;
      }
    }

    pub_->publish(filtered);
  }

  double angle_begin_deg_;
  double angle_end_deg_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
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
