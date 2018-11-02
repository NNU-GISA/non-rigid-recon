#include<iostream>
#include<Kinect.h>
#include<opencv2/opencv.hpp>
#include<vector>
#include<stdlib.h>
#include<string>

using namespace std;
using namespace cv;

int main123(void)
{

	IKinectSensor   *mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IDepthFrameSource *depSource = nullptr;
	mySensor->get_DepthFrameSource(&depSource);
	int depHeight = 0, depWidth = 0;
	IFrameDescription *depDescription = nullptr;
	depSource->get_FrameDescription(&depDescription);
	depDescription->get_Height(&depHeight);
	depDescription->get_Width(&depWidth);

	IDepthFrameReader *depReader = nullptr;
	depSource->OpenReader(&depReader);
	Mat depImg(depHeight, depWidth, CV_16UC1);
	IDepthFrame  *depFrame = nullptr;

	ICoordinateMapper* m_pCoordinateMapper;
	mySensor->get_CoordinateMapper(&m_pCoordinateMapper);



	while (1)
	{
		if (depReader->AcquireLatestFrame(&depFrame) == S_OK)
		{
			depFrame->CopyFrameDataToArray(depHeight * depWidth, (UINT16 *)depImg.data);
			imshow("depth", depImg * 16);
			depFrame->Release();
		}
		char c = waitKey(1);
		if (c == VK_ESCAPE)
			break;
	}

	CameraIntrinsics* m_pCameraIntrinsics = new CameraIntrinsics[0];
	// 获取深度相机内参并打印
	m_pCoordinateMapper->GetDepthCameraIntrinsics(m_pCameraIntrinsics);
	cout << "FocalLengthX : " << m_pCameraIntrinsics->FocalLengthX << endl;
	cout << "FocalLengthY : " << m_pCameraIntrinsics->FocalLengthY << endl;
	cout << "PrincipalPointX : " << m_pCameraIntrinsics->PrincipalPointX << endl;
	cout << "PrincipalPointY : " << m_pCameraIntrinsics->PrincipalPointY << endl;
	cout << "RadialDistortionFourthOrder : " << m_pCameraIntrinsics->RadialDistortionFourthOrder << endl;
	cout << "RadialDistortionSecondOrder : " << m_pCameraIntrinsics->RadialDistortionSecondOrder << endl;
	cout << "RadialDistortionSixthOrder : " << m_pCameraIntrinsics->RadialDistortionSixthOrder << endl;

	depReader->Release();
	depDescription->Release();
	depSource->Release();
	mySensor->Close();
	mySensor->Release();

	system("pause");
	return 1;
}