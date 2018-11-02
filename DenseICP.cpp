#include "DenseICP.hpp"
void EstimateTransMatBigStep2(
	pcl::PointCloud<pointT>::Ptr src_Ptcloud,
	pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
	pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
	Eigen::Matrix4f &transMat)
{
	pcl::IterativeClosestPoint<pointT, pointT> icp;
	icp.setInputSource(src_Ptcloud);
	icp.setInputTarget(tar_Ptcloud);
	icp.setMaxCorrespondenceDistance(12);
	icp.setMaximumIterations(20);
	icp.setTransformationEpsilon(1e-5);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.align(*registrated_Ptcloud);
	transMat = icp.getFinalTransformation();
}

void DenseICP::EstimateTransMat(
pcl::PointCloud<pointT>::Ptr src_Ptcloud,
pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
Eigen::Matrix4f &transMat)
{

	pcl::PointCloud<pointT>::Ptr registrated_Ptcloud0(new pcl::PointCloud<pointT>);
	Eigen::Matrix4f transMat0;
	this->EstimateTransMatBigStep(src_Ptcloud, tar_Ptcloud, registrated_Ptcloud0, transMat0);

	//Eigen::Matrix4f transMat1;
	//this->EstimateTransMatBigStep(registrated_Ptcloud0, tar_Ptcloud, registrated_Ptcloud, transMat1);
	transMat = transMat0;
	/*
    pcl::IterativeClosestPoint<pointT,pointT> icp;
    icp.setInputSource(src_Ptcloud);
    icp.setInputTarget(tar_Ptcloud);
    icp.setMaxCorrespondenceDistance (10);
    icp.setMaximumIterations (50);
    icp.setTransformationEpsilon (1e-5);
    icp.setEuclideanFitnessEpsilon (0.01);
    icp.align(*registrated_Ptcloud);*/
    //transMat = icp.getFinalTransformation();
}
void DenseICP::EstimateTransMatMM(
	pcl::PointCloud<pointT>::Ptr src_Ptcloud,
	pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
	pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
	Eigen::Matrix4f &transMat)
{

	pcl::PointCloud<pointT>::Ptr registrated_Ptcloud0(new pcl::PointCloud<pointT>);
	Eigen::Matrix4f transMat0;
	EstimateTransMatBigStep2(src_Ptcloud, tar_Ptcloud, registrated_Ptcloud0, transMat0);

	Eigen::Matrix4f transMat1;
	this->EstimateTransMatSmallStep(registrated_Ptcloud0, tar_Ptcloud, registrated_Ptcloud, transMat1);
	transMat = transMat1* transMat0;
}

void DenseICP::EstimateTransMatBigStep(
	pcl::PointCloud<pointT>::Ptr src_Ptcloud,
	pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
	pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
	Eigen::Matrix4f &transMat)
{
	pcl::IterativeClosestPoint<pointT, pointT> icp;
	icp.setInputSource(src_Ptcloud);
	icp.setInputTarget(tar_Ptcloud);
	icp.setMaxCorrespondenceDistance(0.050);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1);
	icp.align(*registrated_Ptcloud);
	transMat = icp.getFinalTransformation();
}

void DenseICP::EstimateTransMatSmallStep(
	pcl::PointCloud<pointT>::Ptr src_Ptcloud,
	pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
	pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
	Eigen::Matrix4f &transMat)
{
	pcl::IterativeClosestPoint<pointT, pointT> icp;
	icp.setInputSource(src_Ptcloud);
	icp.setInputTarget(tar_Ptcloud);
	icp.setMaxCorrespondenceDistance(8);
	icp.setMaximumIterations(50);
	icp.setTransformationEpsilon(1e-5);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.align(*registrated_Ptcloud);
	transMat = icp.getFinalTransformation();
}
bool IsInsideTSDF(const tsdfBound &bound, Eigen::Vector3f worldPt)
{
	if (worldPt(0) >= bound.x_down_bound && worldPt(0) <= bound.x_up_bound)
	{
		if (worldPt(1) >= bound.y_down_bound && worldPt(1) <= bound.y_up_bound)
		{
			if (worldPt(2) >= bound.z_down_bound && worldPt(2) <= bound.z_up_bound)
			{
				return true;
			}
		}
	}
	return false;
}
void  DenseICP::GetDensePtCloud(pcl::PointCloud<pointT>::Ptr ptCloud,const tsdfBound &bound,
	int image_width, int image_height, float *depth, const Eigen::Matrix3f &camera_intr)
{
	for (int w = 0; w < image_width; w++)
	{
		for (int h = 0; h < image_height; h++)
		{
			Eigen::Vector3f imgPt(w, h, 1);
			Eigen::Vector3f worldPt = depth[h*image_width + w] * camera_intr.inverse() * imgPt;
			if (IsInsideTSDF(bound, worldPt))
			{
				pcl::PointXYZ pt;
				pt.x = worldPt(0);
				pt.y = worldPt(1);
				pt.z = worldPt(2);
				ptCloud->push_back(pt);
			}
		}
	}
}