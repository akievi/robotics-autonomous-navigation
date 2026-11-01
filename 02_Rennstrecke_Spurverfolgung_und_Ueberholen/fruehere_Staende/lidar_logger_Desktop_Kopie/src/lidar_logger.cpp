#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <ctime>

/* OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <math.h>


using namespace cv;
using namespace std;

using std::placeholders::_1;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------

class lidar_logger : public rclcpp::Node
{
public:
	lidar_logger() : Node("lidar_logger_node")
	{
  	// Subscriptions

  	// Lidar and IMU
  	sub_lidar_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1, 			std::bind(&lidar_logger::callbackLidarScan, this, _1));
	  //sub_imu = this->create_subscription<std_msgs::msg::Float32MultiArray>("/imu_euler", 1, 
  	//std::bind(&lidar_logger::callbackImu, this, _1));

  	// Timer for periodic tasks
  	//timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&lidar_logger::timerCallback, 	this));
	}

	void callbackLidarScan(const sensor_msgs::msg::LaserScan &scan)
  	{
    		//Define laser image and image size
    		int img_size = 400;
    		cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC3);
    		cv::Point center(img_size / 2, img_size / 2);
    
    		//Scaling factor to fit the LaserScan data within the image
    		//float scale = img_size / (2.0f * scan.range_max);
    		float scale = img_size / (2.0f * 2); // see only a circle of 2 meters
    		
    		//scan_points = new float[sca];
    		//Iterate through laser scan ranges
    		for (size_t i = 0; i < scan.ranges.size(); ++i) {
    			float range = scan.ranges[i];
    			if(std::isfinite(range)){
    				float angle = scan.angle_min + i * scan.angle_increment;
    				int x = static_cast<int>(center.x + range * scale * std::cos(angle));
    				int y = static_cast<int>(center.y + range * scale * std::sin(angle));
    				//float degree = RAD2DEG(angle);
    				//Draw the point
    				if (angle > 0)
	    				cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0,0,255), -1);
	    			else if (angle < 0)
	    				cv::circle(image, cv::Point(x,y), 2, cv::Scalar(0,255,255), -1);
    			}
    		}	
    
	    	//Process the Text that displays the IMU Message
	    	//std::string imu_text = to_string(yaw);
	    	//cv::Point position(10,30);
	    	//int fontType = cv::FONT_HERSHEY_SIMPLEX;
	    	//double fontScale = 1.0;
	    	//cv::Scalar color(255, 255, 255);
	    	//int thickness = 2;
	    	//int lineType = cv::LINE_AA;
	    	//cv::putText(image, imu_text, position, fontType, fontScale, color, thickness,lineType);
	    
	    	//Draw an Arrow for the IMU
	    	//cv::Point startPoint(img_size / 2, img_size / 2);
	    	//double length = 100;
	    	//double angleInRadians = yaw * CV_PI / 180.0;
	    	//cv::Point endPoint(
	    		//startPoint.x + static_cast<int>(length * std::cos(angleInRadians)),
	    		//startPoint.y + static_cast<int>(length * std::sin(angleInRadians))
	    		//);
	    	//cv::arrowedLine(image, startPoint, endPoint, cv::Scalar(0,255, 0), thickness, lineType);
	    
	    	//Copy image to cv_bridge image to be sent as a ROS message
	    	//cv_image.image = image.clone();
	    	//Display the image
	    	cv::imshow("LaserScan", image);
	    	cv::waitKey(10);
   	}

	/*void lidar_logger::callbackImu(const std_msgs::msg::Float32MultiArray::SharedPtr imu_euler)
	{
	   yaw = imu_euler->data[0]; // IMU_Euler Data
	  // cout << "yaw = " << yaw << endl;

	  if (yaw_init == -1.0)
	  {
	    yaw_init = yaw;
	  }
	}*/
	
	private:
	 	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan;
    		rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_imu;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_logger>());
  rclcpp::shutdown();

  return (0);
}
