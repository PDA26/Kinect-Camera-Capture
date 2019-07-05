#pragma once

#include <string>

// Kinect SDK
#include <Kinect.h>

class KinectCamera
{
public:
	KinectCamera();
	~KinectCamera();

	bool m_bColorWanted;
	bool m_bInfraredWanted;
	bool m_bDepthWanted;
	std::string m_szColorOutput;
	std::string m_szInfraredOutput;
	std::string m_szDepthOutput;

	HRESULT InitializeDefaultSensor();
	HRESULT CheckAvailiable();
	HRESULT Capture();

private:
	HRESULT ProcessColor(IColorFrame * pColorFrame);
	HRESULT ProcessInfrared(IInfraredFrame * pInfraredFrame);
	HRESULT ProcessDepth(IDepthFrame * pDepthFrame);

    // Current Kinect
    IKinectSensor* m_pKinectSensor;

	// Multi reader
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;

};

