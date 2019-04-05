#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "KinectAPI.hpp"
#include <std_msgs/Float64MultiArray.h>

cv::Mat getDepthDrawableImage(cv::Mat depth_map) {
	cv::Mat drawable;
	depth_map.convertTo(drawable, CV_8UC3, 255 / 4000.0);
	cv::cvtColor(drawable, drawable, cv::COLOR_GRAY2BGR);
	return drawable;
}

int main(int argc, char** argv) {

	chdir("/home/migly/catkin_ws/src/ros_kinect");

	ros::init(argc, argv, "ros_kinect");

	ros::NodeHandle n;

	KinectAPI KinectAPI("User Viewer");

	int max_index = 0;
	cv::Point3d input_points[16];
	std_msgs::Float64MultiArray user_points;
	ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ros_kinect/points", 1000);
	ros::Publisher color = n.advertise<sensor_msgs::Image>("/ros_kinect/color", 1000);
	ros::Publisher depth = n.advertise<sensor_msgs::Image>("/ros_kinect/depth", 1000);

	while (ros::ok()) {
		KinectAPI.update(&max_index, input_points);
		if (!KinectAPI.getColorImage().empty() && !KinectAPI.getDepthImage().empty() && !KinectAPI.getUserTracker().empty()) {
			//cv::imshow("color", KinectAPI.getColorImage());
			//cv::imshow("depth", KinectAPI.getDepthDrawableImage(KinectAPI.getDepthImage()));
			cv::imshow("user", KinectAPI.getUserTracker());

			//cv::Mat result = KinectAPI.getUserTracker() + KinectAPI.getDepthDrawableImage(KinectAPI.getDepthImage());
			cv::Mat result = KinectAPI.getSkeletonImage() + getDepthDrawableImage(KinectAPI.getDepthImage());
			user_points.data.clear();
			for (int i = 0; i < max_index; ++i) {
				//printf("%f %f\n", input_points[i].x, input_points[i].y );
				//cv::circle(result, cv::Point(input_points[i].x, input_points[i].y), 10, cv::Scalar(0, 0, 255), -1);
				user_points.data.push_back(i + 1);
				user_points.data.push_back(input_points[i].x);
				user_points.data.push_back(input_points[i].y);
				user_points.data.push_back(input_points[i].z);
			}
			pub.publish(user_points);

			//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			color.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", KinectAPI.getColorImage()).toImageMsg());
			depth.publish(cv_bridge::CvImage(std_msgs::Header(), "mono16", KinectAPI.getDepthImage()).toImageMsg());
			//cv::imshow("user2", result );
			cv::waitKey(1);
		}
		ros::spinOnce();
	}

	return 0;
}
