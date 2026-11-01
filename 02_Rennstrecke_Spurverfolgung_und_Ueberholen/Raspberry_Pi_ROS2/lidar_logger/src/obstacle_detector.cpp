#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector")
  {
    detection_distance_ =
      declare_parameter<double>("detection_distance", 0.50);

	// QoS compatible with RViz (RELIABLE)
    	auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliable()
                 .durability_volatile();
                 
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_filtered",
      qos,
      std::bind(
        &ObstacleDetectorNode::scan_callback,
        this,
        std::placeholders::_1));

    obstacle_pub_ = create_publisher<std_msgs::msg::Int32>(
      "obstacle_detected",
      10);

    RCLCPP_INFO(
      get_logger(),
      "Obstacle detector started. Threshold = %.2f m",
      detection_distance_);
  }

private:
  void scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    bool obstacle_detected = false;

    for (const auto & range : msg->ranges)
    {
      // Ignore invalid measurements
      if (!std::isfinite(range))
      {
        continue;
      }

      if (range < detection_distance_)
      {
        obstacle_detected = true;
        break;
      }
    }

    std_msgs::msg::Int32 output_msg;
    output_msg.data = obstacle_detected ? 1 : 0;

    obstacle_pub_->publish(output_msg);
  }

  double detection_distance_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr obstacle_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ObstacleDetectorNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
