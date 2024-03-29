// KinectPhotoCaptureTest.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "Log.h"
#include "KinectCamera.h"

int main()
{
	Log::Initialise("Test.log");
	Log::SetThreshold(Log::Level::Debug);
	KinectCamera TestCamera;

	// 设置保存图片的路径
	TestCamera.m_szColorOutput = "TestColor.png";
	TestCamera.m_szInfraredOutput = "TestInfrared.png";
	TestCamera.m_szDepthOutput = "TestDepth.png";

	// 初始化相机（驱动安装正确即可成功，无论相机是否连接）
	TestCamera.InitializeDefaultSensor();
	
	// 检测相机是否可用（相机未连接会导致失败）
	std::uintmax_t cnt = 0;
	for (cnt = 0; FAILED(TestCamera.CheckAvailiable()); cnt++)
		Sleep(100);
	std::cout << cnt << '\n';

	cnt = 0;
	for (int i = 0; i < 100; i++)
	{
		// 采集图像
		if (SUCCEEDED(TestCamera.Capture()))
			cnt++;
	}

	std::cout << cnt << " times succeeded\n";

    return 0;
}

