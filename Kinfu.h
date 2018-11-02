#pragma once
#include "DenseICP.hpp"
#include "TSDF.h"
#include <string>
#include <vector>
class Kinfu
{
public:
	Kinfu();
	~Kinfu();
	void Initialize();
	void Build();
	void GetDepthFromImg(std::string imgfile, float *depth, float scale);
	void SetCameraIntr(const Eigen::Matrix3f &cameraIntr);

public:
	Eigen::Matrix3f _cameraIntr;
	std::vector<std::string> _imgFileLists;
};

