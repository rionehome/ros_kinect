#include <iostream>
#include "NiTE.h"

#define MAX_DEPTH 10000

class HumanTracker {
public:
	HumanTracker(const char* strSampleName);
	virtual ~HumanTracker();

	virtual openni::Status Init(int argc, char **argv);
	void Display();

protected:
	void Finalize();

private:
	HumanTracker(const HumanTracker&);
	HumanTracker& operator=(HumanTracker&);

	static HumanTracker* ms_self;

	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	//unsigned int		m_nTexMapY;

	openni::Device		m_device;
	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
};

HumanTracker* HumanTracker::ms_self = NULL;

bool g_drawDepth = true;

HumanTracker::HumanTracker(const char* strSampleName) : m_poseUser(0) {
	ms_self = this;
	m_pUserTracker = new nite::UserTracker;
}
HumanTracker::~HumanTracker() {
	printf("fin\n");
	Finalize();

	delete[] m_pTexMap;

	ms_self = NULL;
}

void HumanTracker::Finalize() {
	delete m_pUserTracker;
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

openni::Status HumanTracker::Init(int argc, char **argv) {
	m_pTexMap = NULL;

	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK) {
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	const char* deviceUri = openni::ANY_DEVICE;
	for (int i = 1; i < argc - 1; ++i) {
		if (strcmp(argv[i], "-device") == 0) {
			deviceUri = argv[i + 1];
			break;
		}
	}

	rc = m_device.open(deviceUri);
	if (rc != openni::STATUS_OK) {
		printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
		return rc;
	}

	nite::NiTE::initialize();

	if (m_pUserTracker->create(&m_device) != nite::STATUS_OK) {
		return openni::STATUS_ERROR;
	}
	return openni::STATUS_OK;
}

void HumanTracker::Display() {
	printf("display\n");
	nite::UserTrackerFrameRef userTrackerFrame;
	openni::VideoFrameRef depthFrame;
	openni::VideoFrameRef colorFrame;
	nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);
	if (rc != nite::STATUS_OK) {
		printf("GetNextData failed\n");
		return;
	}

	depthFrame = userTrackerFrame.getDepthFrame();

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

	// check if we need to draw depth frame to texture
	if (depthFrame.isValid()) {
		const nite::UserId* pLabels = userLabels.getPixels();

		const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		//printf("%d\n", depthFrame.getHeight());
		//printf("%d\n", depthFrame.getWidth());
		for (int y = 0; y < depthFrame.getHeight(); ++y) {
			const openni::DepthPixel* pDepth = pDepthRow;



			for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth,  ++pLabels) {

				//std::cout << (char*)depthFrame.getData() << '\n';

				if (*pDepth != 0) {
					//printf("%f\n", (*pDepth) / 1000.0 );
					if (*pLabels == 0) {

					} else {
						//人間の描画
						// /printf("%d", *pLabels);
					}
				}
			}
		}
	}
	printf("display2\n");
}