#include "Kinfu.h"

#include <opencv2\opencv.hpp>
#include <opencv2\highgui.hpp>




Kinfu::Kinfu()
{
}


Kinfu::~Kinfu()
{

}

void Kinfu::GetDepthFromImg(std::string imgfile, float *depth, float scale)
{
	cv::Mat map = cv::imread(imgfile, CV_LOAD_IMAGE_UNCHANGED);
	//cv::imshow("window", map);
	int width = map.cols;
	int height = map.rows;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			ushort d = map.at<ushort>(i, j);
			depth[width*i + j] = float(d * scale);
		}
	}
}
void Kinfu::SetCameraIntr(const Eigen::Matrix3f &cameraIntr)
{
	this->_cameraIntr = cameraIntr;
}