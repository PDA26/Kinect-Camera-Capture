#include "KinectCamera.h"

// Log
#include "Log.h"

// OpenCV for image saving
#include <opencv2/opencv.hpp>

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
    if (pInterfaceToRelease != NULL)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}

KinectCamera::KinectCamera()
	: m_pKinectSensor(nullptr)
	, m_pMultiSourceFrameReader(nullptr)
	, m_bColorWanted(true)
	, m_bInfraredWanted(true)
	, m_bDepthWanted(true)
{
}


KinectCamera::~KinectCamera()
{
	SafeRelease(m_pMultiSourceFrameReader);
	if (m_pKinectSensor != nullptr)
		m_pKinectSensor->Close();
	SafeRelease(m_pKinectSensor);
}

HRESULT KinectCamera::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the frame reader

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			DWORD WantedTypes = FrameSourceTypes::FrameSourceTypes_None;
			if (m_bColorWanted)
				WantedTypes |= FrameSourceTypes::FrameSourceTypes_Color;
			if (m_bInfraredWanted)
				WantedTypes |= FrameSourceTypes::FrameSourceTypes_Infrared;
			if (m_bDepthWanted)
				WantedTypes |= FrameSourceTypes::FrameSourceTypes_Depth;
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				WantedTypes,
				&m_pMultiSourceFrameReader);
		}
	}
	else
		hr = E_FAIL;

	if (FAILED(hr))
		Log::Error("No ready Kinect found!");
	else
		Log::Info("Kinect initialized successfully");

    return hr;
}

HRESULT KinectCamera::CheckAvailiable()
{
	BOOLEAN bIsAvailable = FALSE;

	m_pKinectSensor->get_IsAvailable(&bIsAvailable);

	if (!bIsAvailable)
	{
		// No real Kinect connected
		// or Kinect not ready
		return E_FAIL;
	}
	return S_OK;
}

HRESULT KinectCamera::Capture()
{
	if (!m_pMultiSourceFrameReader)
	{
		Log::Error("Kinect did not initialize correctly");
		return E_NOT_VALID_STATE;
	}

	IMultiSourceFrame* pMultiSourceFrame = nullptr;

	int i;
	for (i = 0; FAILED(m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame)); i++)
		SafeRelease(pMultiSourceFrame);
	Log::Debug("Acquire MultiSourceFrame: Tried %d times", i);

	HRESULT hrColor, hrInfrared, hrDepth;
	IColorFrameReference* pColorFrameReference = nullptr;
	IColorFrame* pColorFrame = nullptr;
	IInfraredFrameReference* pInfraredFrameReference = nullptr;
	IInfraredFrame* pInfraredFrame = nullptr;
	IDepthFrameReference* pDepthFrameReference = nullptr;
	IDepthFrame* pDepthFrame = nullptr;

	// MultiSourceFrame 获取成功后立即获取每个 Frame ，否则失败率很高
	if (m_bColorWanted)
	{
		pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		hrColor = pColorFrameReference->AcquireFrame(&pColorFrame);
	}
	if (m_bInfraredWanted)
	{
		pMultiSourceFrame->get_InfraredFrameReference(&pInfraredFrameReference);
		hrInfrared = pInfraredFrameReference->AcquireFrame(&pInfraredFrame);
	}
	if (m_bDepthWanted)
	{
		pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		hrDepth = pDepthFrameReference->AcquireFrame(&pDepthFrame);
	}
	// 然后再进行耗时长的操作

	if (m_bColorWanted)
	{
		if (SUCCEEDED(hrColor))
			hrColor = ProcessColor(pColorFrame);
		else
			Log::Error("Failed to acquire color frame: %X", hrColor);
		SafeRelease(pColorFrame);
		SafeRelease(pColorFrameReference);
	}
	if (m_bInfraredWanted)
	{
		if (SUCCEEDED(hrInfrared))
			hrInfrared = ProcessInfrared(pInfraredFrame);
		else
			Log::Error("Failed to acquire infrared frame: %X", hrInfrared);
		SafeRelease(pInfraredFrame);
		SafeRelease(pInfraredFrameReference);
	}
	if (m_bDepthWanted)
	{
		if (SUCCEEDED(hrDepth))
			hrDepth = ProcessDepth(pDepthFrame);
		else
			Log::Error("Failed to acquire depth frame: %X", hrDepth);
		SafeRelease(pDepthFrame);
		SafeRelease(pDepthFrameReference);
	}

	SafeRelease(pMultiSourceFrame);

	if (SUCCEEDED(hrColor) && SUCCEEDED(hrInfrared) && SUCCEEDED(hrDepth))
		return S_OK;
	else
		return E_FAIL;
}

HRESULT KinectCamera::ProcessColor(IColorFrame * pColorFrame) 
{
	IFrameDescription* pFrameDescription = nullptr;
	int nWidth = 0;
	int nHeight = 0;
	HRESULT hr;

	pColorFrame->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&nWidth);
	pFrameDescription->get_Height(&nHeight);
	SafeRelease(pFrameDescription);

	cv::Mat img(nHeight, nWidth, CV_8UC4);
	hr = pColorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(img.total() *
																	  img.elemSize() /
																	  img.elemSize1()),
													img.data,
													ColorImageFormat_Bgra);
	if (SUCCEEDED(hr))
	{
		if (cv::imwrite(m_szColorOutput, img))
		{
			Log::Info("Color frame saved to " + m_szColorOutput);
		}
		else
		{
			Log::Error("Failed to write color frame");
		}
	}
	else
	{
		Log::Error("Failed to get color frame: %X", hr);
	}

	return hr;
}

HRESULT KinectCamera::ProcessInfrared(IInfraredFrame * pInfraredFrame)
{
	IFrameDescription* pFrameDescription = nullptr;
	int nWidth = 0;
	int nHeight = 0;
	HRESULT hr;

	pInfraredFrame->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&nWidth);
	pFrameDescription->get_Height(&nHeight);
	SafeRelease(pFrameDescription);

	cv::Mat img(nHeight, nWidth, CV_16U);
	hr = pInfraredFrame->CopyFrameDataToArray(static_cast<UINT>(img.total() *
																img.elemSize() /
																img.elemSize1()),
											  reinterpret_cast<UINT16*>(img.data));

	if (SUCCEEDED(hr))
	{
		if (cv::imwrite(m_szInfraredOutput, img))
		{
			Log::Info("Infrared frame saved to " + m_szInfraredOutput);
		}
		else
		{
			Log::Error("Failed to write infrared frame");
		}
	}
	else
	{
		Log::Error("Failed to get infrared frame: %X", hr);
	}
	return hr;
}

HRESULT KinectCamera::ProcessDepth(IDepthFrame * pDepthFrame)
{
	IFrameDescription* pFrameDescription = nullptr;
	int nWidth = 0;
	int nHeight = 0;
	HRESULT hr;

	pDepthFrame->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&nWidth);
	pFrameDescription->get_Height(&nHeight);
	SafeRelease(pFrameDescription);

	cv::Mat img(nHeight, nWidth, CV_16U);
	hr = pDepthFrame->CopyFrameDataToArray(static_cast<UINT>(img.total() *
															 img.elemSize() /
															 img.elemSize1()),
										   reinterpret_cast<UINT16*>(img.data));

	if (SUCCEEDED(hr))
	{
		if (cv::imwrite(m_szDepthOutput, img))
		{
			Log::Info("Depth frame saved to " + m_szDepthOutput);
		}
		else
		{
			Log::Error("Failed to write depth frame");
		}
    }
	else
	{
		Log::Error("Failed to get depth frame: %X", hr);
	}
	return hr;
}
