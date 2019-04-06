#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "KinectAPI.hpp"
#include <std_msgs/Float64MultiArray.h>

cv::Mat getDepthDrawableImage(std::vector<double> depth_vector) {
	cv::Mat raw_input = cv::Mat::zeros(480, 640, CV_64F);
	cv::Mat drawable;
	for (int x = 0; x < 640; ++x) {
		for (int y = 0; y < 480; ++y) {
			raw_input.at<double>(y, x) = depth_vector[640 * y + x] * 255 / 4000.0;
		}
	}
	raw_input.convertTo(drawable, CV_8UC1);
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
	ros::Publisher pub_points = n.advertise<std_msgs::Float64MultiArray>("/ros_kinect/points", 1000);
	ros::Publisher pub_color = n.advertise<sensor_msgs::Image>("/ros_kinect/color", 1000);
	ros::Publisher pub_depth = n.advertise<std_msgs::Float64MultiArray_<double>>("/ros_kinect/depth", 1000);
	std_msgs::Float64MultiArray depth_array;
	while (ros::ok()) {
		KinectAPI.update(&max_index, input_points);
		if (!KinectAPI.getColorImage().empty() && !KinectAPI.getDepthVector().empty() && !KinectAPI.getUserTracker().empty()) {
			//cv::imshow("color", KinectAPI.getColorImage());
			cv::imshow("depth", getDepthDrawableImage(KinectAPI.getDepthVector()));
			//cv::imshow("user", KinectAPI.getUserTracker());
			//cv::Mat result = KinectAPI.getColorImage() + getDepthDrawableImage(KinectAPI.getDepthVector());
			//cv::Mat result = KinectAPI.getUserTracker() + getDepthDrawableImage(KinectAPI.getDepthVector());
			//cv::Mat result = KinectAPI.getSkeletonImage() + getDepthDrawableImage(KinectAPI.getDepthVector());
			user_points.data.clear();
			for (int i = 0; i < max_index; ++i) {
				//printf("%f %f\n", input_points[i].x, input_points[i].y );
				//cv::circle(result, cv::Point(input_points[i].x, input_points[i].y), 10, cv::Scalar(0, 0, 255), -1);
				user_points.data.push_back(i + 1);
				user_points.data.push_back(input_points[i].x);
				user_points.data.push_back(input_points[i].y);
				user_points.data.push_back(input_points[i].z);
			}
			pub_points.publish(user_points);

			depth_array.data.clear();
			for (double depth : KinectAPI.getDepthVector()) depth_array.data.push_back(depth);

			//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
			pub_color.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", KinectAPI.getColorImage()).toImageMsg());
			pub_depth.publish(depth_array);
			//depth.publish(cv_bridge::CvImage(std_msgs::Header(), "mono64", KinectAPI.getDepthVector()).toImageMsg());
			//cv::imshow("user2", result );
			cv::waitKey(1);
		}
		ros::spinOnce();
	}

	return 0;
}
