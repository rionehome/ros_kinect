#include <ros/ros.h>
#include <iostream>
#include "NiTE.h"
#include <opencv2/opencv.hpp>
#include <bitset>
#include <string>


using namespace cv;
using namespace std;

//ユーザーの色分け用カラーコード
int Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {1, 0, 1}, {0, 1, 1}, {1, 1, 1}};

cv::Mat color;
cv::Mat depth_map;
cv::Mat user;
cv::Mat skeleton;

class KinectAPI {
public:
	KinectAPI(const char* strSampleName);
	~KinectAPI();
	openni::Status Init();
	void update(int *max_index, cv::Point3d points[]);
	cv::Mat getUserTracker() {return user;}
	cv::Mat getColorImage() {return color;}
	cv::Mat getDepthImage() {return depth_map;}
	cv::Mat getSkeletonImage() {return skeleton;}

private:
	nite::Status rc_n;
	nite::UserId m_poseUser;
	nite::UserTrackerFrameRef userTrackerFrame;
	nite::UserTracker* m_pUserTracker;

	openni::Status rc_o;
	openni::Device m_device;
	openni::VideoStream color_stream;
	openni::VideoStream depth_stream;
	openni::VideoFrameRef depth_frame;
	openni::VideoFrameRef color_frame;

	std::vector<openni::VideoStream*> streams;
};


//////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat getColorImage_src(openni::VideoFrameRef& color_frame) {
	if (!color_frame.isValid()) {
		return cv::Mat();
	}
	openni::VideoMode video_mode = color_frame.getVideoMode();
	cv::Mat color_img = cv::Mat(video_mode.getResolutionY(),
	                            video_mode.getResolutionX(),
	                            CV_8UC3, (char*)color_frame.getData());
	cv::Mat ret_img;
	cv::cvtColor(color_img, ret_img, CV_BGRA2RGB);
	return ret_img;
}

cv::Mat getDepthMap_src(openni::VideoFrameRef& depth_frame) {
	if (!depth_frame.isValid()) {
		return cv::Mat();
	}
	cv::Mat map(depth_frame.getVideoMode().getResolutionY(), depth_frame.getVideoMode().getResolutionX(), CV_MAKETYPE(CV_64F, 1));

	const openni::DepthPixel* depth_raw = (const openni::DepthPixel*)depth_frame.getData();

	for (int y = 0; y < depth_frame.getVideoMode().getResolutionY(); ++y) {
		for (int x = 0; x < depth_frame.getVideoMode().getResolutionX(); ++x, ++depth_raw) {
			map.at<double>(y, x) = *depth_raw;
		}
	}
	return map;
}

cv::Mat getUserTracker_src(openni::VideoFrameRef& depth_frame, const nite::UserMap& userLabels, int *max_index, cv::Point3d points[]) {

	int count[16] = {0};
	*max_index = 0;
	for (int i = 0; i < 16; ++i) {
		points[i].x = 0.0;
		points[i].y = 0.0;
		points[i].z = 0.0;
	}
	if (!depth_frame.isValid()) {
		return cv::Mat();
	}
	cv::Mat id_map = Mat(depth_frame.getVideoMode().getResolutionY(), depth_frame.getVideoMode().getResolutionX(), CV_8UC3);

	const nite::UserId* pLabels = userLabels.getPixels();

	for (int y = 0; y < depth_frame.getVideoMode().getResolutionY(); ++y) {
		for (int x = 0; x < depth_frame.getVideoMode().getResolutionX(); ++x, ++pLabels) {
			if (*pLabels == 0) {
				id_map.at<Vec3b>(y, x) = Vec3b(0, 0, 0);
			} else {
				//人間の描画
				id_map.at<Vec3b>(y, x) = Vec3b(Colors[(*pLabels) % 8][0] * 255, Colors[(*pLabels) % 8][1] * 255, Colors[(*pLabels) % 8][2] * 255);
				//重心計算
				points[*pLabels - 1].x += x;
				points[*pLabels - 1].y += y;
				points[*pLabels - 1].z += depth_map.at<double>(y, x);
				count[*pLabels - 1]++;
				if (*max_index < *pLabels) *max_index = *pLabels;
			}
		}
	}

	if (!depth_map.empty()) {
		for (int i = 0; i < *max_index; ++i) {
			if (count[i] != 0) {
				points[i].x = points[i].x / count[i];
				points[i].y = points[i].y / count[i];
				points[i].z = points[i].z / count[i];
			}
		}
	}
	cv::cvtColor(id_map, id_map, CV_RGB2BGR);
	return id_map;
}

void getJoint(cv::Mat *input, nite::UserTracker* m_pUserTracker, const nite::Point3f joint, cv::Point3d *point) {
	float convertX = 0;
	float convertY = 0;
	m_pUserTracker->convertJointCoordinatesToDepth(joint.x, joint.y, joint.z, &convertX, &convertY);
	cv::circle(*input, cv::Point((int)convertX, (int)convertY), 10, cv::Scalar(0, 255, 0), -1);
	point->x += convertX;
	point->y += convertY;
	point->z += joint.z;
}

cv::Mat getUserSkeleton_src(openni::VideoFrameRef& depth_frame, const nite::Array<nite::UserData>& users, nite::UserTracker* m_pUserTracker, int *max_index, cv::Point3d points[]) {
	cv::Mat skeleton_map = Mat::zeros(cv::Size(depth_frame.getVideoMode().getResolutionX(), depth_frame.getVideoMode().getResolutionY()), CV_8UC3);

	for (int i = 0; i < users.getSize(); ++i) {
		const nite::UserData& user = users[i];
		if (user.isNew()) {
			m_pUserTracker->startSkeletonTracking(user.getId());
			m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
		} else if (!user.isLost()) {
			if (user.getSkeleton().getState() == nite::SKELETON_TRACKED) {
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_HEAD).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_TORSO).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition(), &points[i]);
				getJoint(&skeleton_map, m_pUserTracker, user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition(), &points[i]);
				points[i].x = points[i].x / 14;
				points[i].y = points[i].y / 14;
				points[i].z = points[i].z / 14;
			}
		}
	}
	return skeleton_map;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

//コンストラクタ
KinectAPI::KinectAPI(const char* strSampleName) : m_poseUser(0) {
	rc_o = openni::STATUS_OK;
	m_pUserTracker = new nite::UserTracker;
	//printf("%s\n", argv );
	rc_o = Init();
	if (rc_o != openni::STATUS_OK) {
		printf("何らかの問題が発生しました。\n");
		exit(1);
	}
	streams.push_back(&color_stream);
	streams.push_back(&depth_stream);
	m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
}

//デコンストラクタ
KinectAPI::~KinectAPI() {
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
	printf("終了しました。\n");
}

openni::Status KinectAPI::Init() {

	rc_o = openni::OpenNI::initialize();

	cout << "After initialization:" << openni::OpenNI::getExtendedError();
	const char* deviceURI = openni::ANY_DEVICE;

	rc_o = m_device.open(deviceURI);
	if (rc_o != openni::STATUS_OK) {
		cout << "SimpleViewer: Device open failed:" << openni::OpenNI::getExtendedError();
		openni::OpenNI::shutdown();
		return openni::STATUS_ERROR;
	}

	rc_o = depth_stream.create(m_device, openni::SENSOR_DEPTH);
	if (rc_o == openni::STATUS_OK) {
		rc_o = depth_stream.start();
		if (rc_o != openni::STATUS_OK) {
			cerr << "SimpleViewer: Couldn't start depth stream:" << openni::OpenNI::getExtendedError();
			depth_stream.destroy();
		}
	} else {
		cerr << "SimpleViewer: Couldn't find depth stream:" << openni::OpenNI::getExtendedError();
	}

	rc_o = color_stream.create(m_device, openni::SENSOR_COLOR);
	if (rc_o == openni::STATUS_OK) {
		rc_o = color_stream.start();
		if (rc_o != openni::STATUS_OK) {
			cerr << "SimpleViewer: Couldn't find color stream:" << openni::OpenNI::getExtendedError();
			color_stream.destroy();
		}
	} else {
		cerr << "SimpleViewer: Couldn't find color stream:" << openni::OpenNI::getExtendedError();
	}

	if (!depth_stream.isValid() || !color_stream.isValid()) {
		cerr << "SimpleViewer: No valid streams. Exiting" << std::endl;
		openni::OpenNI::shutdown();
		return openni::STATUS_ERROR;
	}

	nite::NiTE::initialize();

	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK) {
		return openni::STATUS_ERROR;
	}
	return openni::STATUS_OK;
}

void KinectAPI::update(int *max_index, cv::Point3d points[]) {

	int changedIndex;
	openni::Status rc = openni::OpenNI::waitForAnyStream(&streams[0], streams.size(), &changedIndex);
	if (rc != openni::STATUS_OK) {
		cerr << ("Wait failed\n");
		return;
	}

	rc_n = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rc_n != nite::STATUS_OK) {
		printf("GetNextData failed\n");
		return;
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

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();


	if (color_frame.isValid()) {
		color = getColorImage_src(color_frame);
	}

	if (depth_frame.isValid()) {
		depth_map = getDepthMap_src(depth_frame);
		user = getUserTracker_src(depth_frame, userLabels, max_index, points);
		skeleton = getUserSkeleton_src(depth_frame, users, m_pUserTracker, max_index, points);
	}
}