#include <iostream>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

using namespace std;


cv::Mat getColorImage(openni::VideoFrameRef& color_frame) {
	if (!color_frame.isValid()) {
		return cv::Mat();
	}
	openni::VideoMode video_mode = color_frame.getVideoMode();
	cv::Mat color_img = cv::Mat(video_mode.getResolutionY(),
	                            video_mode.getResolutionX(),
	                            CV_8UC3, (char*)color_frame.getData());
	cv::Mat ret_img;
	cv::cvtColor(color_img, ret_img, CV_RGB2BGR);
	return ret_img;
}


// CV_16U
cv::Mat getDepthImage(openni::VideoFrameRef& depth_frame) {
	if (!depth_frame.isValid()) {
		return cv::Mat();
	}

	openni::VideoMode video_mode = depth_frame.getVideoMode();
	cv::Mat depth_img = cv::Mat(video_mode.getResolutionY(),
	                            video_mode.getResolutionX(),
	                            CV_16U, (char*)depth_frame.getData());
	return depth_img.clone();
}


cv::Mat getDepthDrawableImage(cv::Mat depth_image) {
	cv::Mat drawable;
	depth_image.convertTo(drawable, CV_8UC1, 255.0 / 10000);
	drawable = 255 - drawable; // 近い方を白に
	return drawable;
}


int main() {
	openni::Device device;
	openni::VideoStream color_stream, depth_stream;
	const char* deviceURI = openni::ANY_DEVICE;

	openni::Status rc = openni::STATUS_OK;

	rc = openni::OpenNI::initialize();

	cout << "After initialization:" << openni::OpenNI::getExtendedError();

	rc = device.open(deviceURI);
	if (rc != openni::STATUS_OK) {
		cout << "SimpleViewer: Device open failed:" << openni::OpenNI::getExtendedError();
		openni::OpenNI::shutdown();
		return 1;
	}

	rc = depth_stream.create(device, openni::SENSOR_DEPTH);
	if (rc == openni::STATUS_OK) {
		rc = depth_stream.start();
		if (rc != openni::STATUS_OK) {
			cerr << "SimpleViewer: Couldn't start depth stream:" << openni::OpenNI::getExtendedError();
			depth_stream.destroy();
		}
	} else {
		cerr << "SimpleViewer: Couldn't find depth stream:" << openni::OpenNI::getExtendedError();
	}

	rc = color_stream.create(device, openni::SENSOR_COLOR);
	if (rc == openni::STATUS_OK) {
		rc = color_stream.start();
		if (rc != openni::STATUS_OK) {
			cerr << "SimpleViewer: Couldn't find color stream:" << openni::OpenNI::getExtendedError();
			color_stream.destroy();
		}
	} else {
		cerr << "SimpleViewer: Couldn't find color stream:" << openni::OpenNI::getExtendedError();
	}

	if (!depth_stream.isValid() || !color_stream.isValid()) {
		cerr << "SimpleViewer: No valid streams. Exiting" << std::endl;
		openni::OpenNI::shutdown();
		return 2;
	}


	std::vector<openni::VideoStream*> streams;
	streams.push_back(&color_stream);
	streams.push_back(&depth_stream);

	openni::VideoFrameRef color_frame;
	openni::VideoFrameRef depth_frame;

	device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

	while (true) {
		int changedIndex;
		openni::Status rc = openni::OpenNI::waitForAnyStream(&streams[0], streams.size(), &changedIndex);
		if (rc != openni::STATUS_OK) {
			cerr << ("Wait failed\n");
			return -1;
		}

		switch (changedIndex) {
		case 0:
			color_stream.readFrame(&color_frame);
			break;

		case 1:
			depth_stream.readFrame(&depth_frame);
			break;

		default:
			cout << "Error in wait" << endl;
		}

		cv::Mat color_img;
		if (color_frame.isValid()) {
			color_img = getColorImage(color_frame);
			cv::imshow("color", color_img);
		}

		cv::Mat depth_img;
		if (depth_frame.isValid()) {
			depth_img = getDepthImage(depth_frame);
			cv::Mat depth_img_debug = getDepthDrawableImage(depth_img);
			cv::imshow("depth", depth_img_debug);
		}

		if (!depth_img.empty() && !color_img.empty()) {
			cv::Mat depth_img_debug = getDepthDrawableImage(depth_img);
			cv::cvtColor(depth_img_debug, depth_img_debug, cv::COLOR_GRAY2BGR);
			cv::Mat debug_img = color_img * 0.5 + depth_img_debug * 0.5;
			cv::imshow("blend", debug_img);
		}

		int key = cv::waitKey(1);
		if ( key == 'q' ) {
			break;
		}
	}
	return 0;
}