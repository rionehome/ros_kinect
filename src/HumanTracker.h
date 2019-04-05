/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#include "NiTE.h"

#define MAX_DEPTH 10000

class HumanTracker {
public:
	HumanTracker(const char* strSampleName);
	virtual ~HumanTracker();

	virtual openni::Status Init(int argc, char **argv);
	//virtual openni::Status Run();	//Does not return
	void Display();

protected:
	virtual void DisplayPostDraw() {};	// Overload to draw over the screen image

	void InitOpenGLHooks();

	void Finalize();

private:
	HumanTracker(const HumanTracker&);
	HumanTracker& operator=(HumanTracker&);

	static HumanTracker* ms_self;
	static void glutIdle();
	static void glutDisplay();

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	openni::Device		m_device;
	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;
};


#endif // _NITE_USER_VIEWER_H_
