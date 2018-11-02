#include "NewtonGuassSolver.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <Eigen/Cholesky> 
#define Max(a,b) ((a)>(b))?(a):(b);
#include "FileIO.h"


NewtonGuassSolver::NewtonGuassSolver(){};
NewtonGuassSolver::~NewtonGuassSolver(){};

NewtonGuassSolver::NewtonGuassSolver(const Eigen::Matrix3f &cameraIntr,
	int image_width, int image_height, float* depth)
{
	SetCameraIntr(cameraIntr);
	SetDepthInfo(image_width, image_height, depth);
}
void NewtonGuassSolver::SetCameraIntr(const Eigen::Matrix3f &cameraIntr)
{
	this->_cameraIntr = cameraIntr;
}

void NewtonGuassSolver::SetDepthInfo(int image_width, int image_height, float* depth)
{
	this->_image_width = image_width;
	this->_image_height = image_height;
	this->_depth = depth;
}
void TransformPt(const Eigen::Vector3f &inpt,
	const Eigen::Matrix4f &transMat, Eigen::Vector3f &outpt)
{
	Matrix3f rotationMat = transMat.block(0, 0, 3, 3);
	Vector3f translationMat = transMat.block(0, 3, 3, 1);
	outpt = rotationMat* inpt + translationMat;
}
void VectorToSkewSyMat(const Eigen::Matrix<double, 3, 1> &w, Eigen::Matrix<double, 3, 3> &skewMat)
{
	skewMat(0, 0) = (0);
	skewMat(0, 1) = -w(2);
	skewMat(0, 2) = w(1);

	skewMat(1, 0) = w(2);
	skewMat(1, 1) = 0;
	skewMat(1, 2) = -w(0);

	skewMat(2, 0) = -w(1);
	skewMat(2, 1) = w(0);
	skewMat(2, 2) = (0);
}
void ConvertSE3ToJacobiMat(const Eigen::Matrix4d &transMat, Eigen::MatrixXd &jacobiMat, double factor = 1000.0)
{
	jacobiMat.setZero();
	Eigen::Matrix<double, 3, 1> dc1 = transMat.block(0, 0, 3, 1);
	Eigen::Matrix<double, 3, 1> dc2 = transMat.block(0, 1, 3, 1);
	Eigen::Matrix<double, 3, 1> dc3 = transMat.block(0, 2, 3, 1);
	Eigen::Matrix<double, 3, 3> skewMatC1;
	VectorToSkewSyMat(dc1, skewMatC1);
	Eigen::Matrix<double, 3, 3> skewMatC2;
	VectorToSkewSyMat(dc2, skewMatC2);
	Eigen::Matrix<double, 3, 3> skewMatC3;
	VectorToSkewSyMat(dc3, skewMatC3);

	Eigen::Matrix<double, 3, 1> dct = transMat.block(0, 3, 3, 1) / factor;
	Eigen::Matrix<double, 3, 3> skewMatCt;
	VectorToSkewSyMat(dct, skewMatCt);

	Eigen::Matrix<double, 3, 3> identityMat;
	identityMat.setIdentity();
	jacobiMat.block(0, 3, 3, 3) = -skewMatC1;
	jacobiMat.block(3, 3, 3, 3) = -skewMatC2;
	jacobiMat.block(6, 3, 3, 3) = -skewMatC3;
	jacobiMat.block(9, 3, 3, 3) = -skewMatCt;

	jacobiMat.block(9, 0, 3, 3) = identityMat;
}
void ConvertPtToJacobiForm(const Eigen::Vector3d &inpt, Eigen::MatrixXd &jacobiPtMat, double number = 1.0)
{
	//Eigen::Matrix<double, 3, 12> identityMat;
	jacobiPtMat.setZero();
	Eigen::Matrix<double, 1, 4> ptMat;
	ptMat(0, 0) = inpt(0);
	ptMat(0, 1) = inpt(1);
	ptMat(0, 2) = inpt(2);
	ptMat(0, 3) = number;
	jacobiPtMat.block(0, 0, 1, 4) = ptMat;
	jacobiPtMat.block(1, 4, 1, 4) = ptMat;
	jacobiPtMat.block(2, 8, 1, 4) = ptMat;
}
//void NewtonGuassSolver::CalculateJacobiDataTermAnalytic(const Eigen::Vector3d &inpt,
//	const Eigen::Vector3d &normal,
//	const Eigen::Matrix4d &transMat,
//	const Eigen::Vector3d &corpt,
//	Eigen::Matrix<double, 1, 6> &derivativeVec)
//{
//	Eigen::Vector3d newinpt = inpt / float(1000);
//	Eigen::Vector3d newcorpt = corpt / float(1000);
//	Eigen::MatrixXd jacobiMat(12, 6);
//	ConvertSE3ToJacobiMat(transMat, jacobiMat);
//	Eigen::MatrixXd jacobiPtMat0(3, 12);
//	ConvertPtToJacobiForm(newinpt.cast<double>(), jacobiPtMat0, 1);
//	Eigen::Vector3d normalTransformed = transMat.block(0, 0, 3, 3) *normal;
//	Eigen::Matrix<double, 1, 3> normalTransformedtrans = normalTransformed.transpose();
//	std::cout << "jacobiPtMat0" << std::endl;
//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 12; j++)
//		{
//			std::cout << jacobiPtMat0(i, j) << " ";
//		}
//		std::cout << std::endl;
//	}
//	Eigen::Matrix<double, 1, 12> derivativeVec00 =
//		normalTransformedtrans * jacobiPtMat0;
//	Eigen::Matrix<double, 1, 6> derivativeVec0 =
//		derivativeVec00 * jacobiMat;
//
//	Eigen::MatrixXd jacobiPtMat1(3, 12);
//	ConvertPtToJacobiForm(normal, jacobiPtMat1, 1);
//	Eigen::MatrixXd jacobiMat1(12, 6);
//	jacobiMat1 = jacobiMat;
//	Eigen::Matrix<double, 3, 3> zeroMat;
//	zeroMat.setZero();
//	jacobiMat1.block(9, 3, 3, 3) = zeroMat;
//
//	Eigen::Vector3d ptTransformed = transMat.block(0, 0, 3, 3) *newinpt + transMat.block(0, 3, 3, 1) / float(1000);
//	Eigen::Vector3d ptDelt = (ptTransformed - newcorpt);
//	Eigen::Matrix<double, 1, 6> derivativeVec1 =
//		((ptDelt.transpose()) * jacobiPtMat1) * jacobiMat1;
//	derivativeVec = (derivativeVec0 + derivativeVec1) *0.01;
//}
void NewtonGuassSolver::CalculateJacobiDataTermAnalytic(const Eigen::Vector3d &inpt,
	const Eigen::Vector3d &normal,
	const Eigen::Matrix4d &transMat,
	const Eigen::Vector3d &corpt,
	Eigen::Matrix<double, 1, 6> &derivativeVec)
{
	Eigen::Vector3d newinpt = inpt / float(1000);
	Eigen::Vector3d newcorpt = corpt / float(1000);
	Eigen::MatrixXd jacobiMat(12, 6);
	ConvertSE3ToJacobiMat(transMat, jacobiMat);
	Eigen::MatrixXd jacobiPtMat0(3, 12);
	ConvertPtToJacobiForm(newinpt.cast<double>(), jacobiPtMat0, 1);
	Eigen::Vector3d normalTransformed = transMat.block(0, 0, 3, 3) *normal;
	Eigen::Matrix<double, 1, 3> normalTransformedtrans = normalTransformed.transpose();
	std::cout << "jacobiPtMat0" << std::endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			std::cout << jacobiPtMat0(i, j) << " ";
		}
		std::cout << std::endl;
	}
	Eigen::Matrix<double, 1, 12> derivativeVec00 =
		normalTransformedtrans * jacobiPtMat0;
	Eigen::Matrix<double, 1, 6> derivativeVec0 =
		derivativeVec00 * jacobiMat;

	Eigen::MatrixXd jacobiPtMat1(3, 12);
	ConvertPtToJacobiForm(normal, jacobiPtMat1, 1);
	Eigen::MatrixXd jacobiMat1(12, 6);
	jacobiMat1 = jacobiMat;
	Eigen::Matrix<double, 3, 3> zeroMat;
	zeroMat.setZero();
	jacobiMat1.block(9, 3, 3, 3) = zeroMat;

	Eigen::Vector3d ptTransformed = transMat.block(0, 0, 3, 3) *newinpt + transMat.block(0, 3, 3, 1) / float(1000);
	Eigen::Vector3d ptDelt = (ptTransformed - newcorpt);
	Eigen::Matrix<double, 1, 6> derivativeVec1 =
		((ptDelt.transpose()) * jacobiPtMat1) * jacobiMat1;
	derivativeVec = (derivativeVec0 + derivativeVec1) *0.01;
}
void NewtonGuassSolver::CalculateJacobiDataTermAnalyticNew(const Eigen::Vector3d &inpt,
	const Eigen::Vector3d &normal,
	const Eigen::Matrix4d &transMat,
	const Eigen::Vector3d &corpt,
	Eigen::Matrix<double, 1, 6> &derivativeVec)
{
	Eigen::Vector3d newinpt = inpt ;
	Eigen::Vector3d newcorpt = corpt ;
	Eigen::MatrixXd jacobiMat(12, 6);
	ConvertSE3ToJacobiMat(transMat, jacobiMat, 1.0);
	Eigen::MatrixXd jacobiPtMat0(3, 12);
	ConvertPtToJacobiForm(newinpt.cast<double>(), jacobiPtMat0, 1);
	Eigen::Vector3d normalTransformed = transMat.block(0, 0, 3, 3) *normal;
	Eigen::Matrix<double, 1, 3> normalTransformedtrans = normalTransformed.transpose();
	//std::cout << "jacobiPtMat0" << std::endl;
	/*for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			std::cout << jacobiPtMat0(i, j) << " ";
		}
		std::cout << std::endl;
	}*/
	Eigen::Matrix<double, 1, 12> derivativeVec00 =
		normalTransformedtrans * jacobiPtMat0;
	Eigen::Matrix<double, 1, 6> derivativeVec0 =
		derivativeVec00 * jacobiMat;

	Eigen::MatrixXd jacobiPtMat1(3, 12);
	ConvertPtToJacobiForm(normal, jacobiPtMat1, 1);
	Eigen::MatrixXd jacobiMat1(12, 6);
	jacobiMat1 = jacobiMat;
	Eigen::Matrix<double, 3, 3> zeroMat;
	zeroMat.setZero();
	jacobiMat1.block(9, 3, 3, 3) = zeroMat;

	Eigen::Vector3d ptTransformed = transMat.block(0, 0, 3, 3) *newinpt + transMat.block(0, 3, 3, 1);
	Eigen::Vector3d ptDelt = (ptTransformed - newcorpt);
	Eigen::Matrix<double, 1, 6> derivativeVec1 =
		((ptDelt.transpose()) * jacobiPtMat1) * jacobiMat1;
	derivativeVec = (derivativeVec0 + derivativeVec1) *0.01;
}

void NewtonGuassSolver::FindCorrespondence(const float3Vec &inPt_cloud, const float3Vec &inPt_normals,
	 const std::vector<float> &inweightVec,
	 const std::vector< Eigen::Matrix4f> &in_transMatVec,
	 float3Vec &outPt_cloud, float3Vec &outPt_normals, 
	 std::vector<float> &weightVec,
	 std::vector< Eigen::Matrix4f> &transMatVec,
	 float3Vec &corPt_cloud)
{
	MatrixXi visiblityMap(this->_image_height, this->_image_width);
	//this->_visiblityMap = MatrixXi(this->_image_height,this->_image_width);
	visiblityMap.setConstant(-1);
	for (int i = 0; i < inPt_cloud.size(); ++i)
	{
		if (isnan(inPt_cloud[i](0)))
		{
			continue;
		}
		Eigen::Vector3f newPt;
		TransformPt(inPt_cloud[i], in_transMatVec[i], newPt);
		Eigen::Vector3f picPt = this->_cameraIntr* newPt;
		if (picPt(2) != 0.0)
		{
			int w = int((picPt(0) / float(picPt(2))) + 0.5);
			int h = int((picPt(1) / float(picPt(2))) + 0.5);
			if (h >= this->_image_height || h < 0)
			{
				continue;
			}
			if (w >= this->_image_width || w < 0)
			{
				continue;
			}
			float delt_depth = this->_depth[h* this->_image_width + w] - inPt_cloud[i](2);
			if (abs(delt_depth) < this->_visibilityLimit)
			{
				if ((visiblityMap)(h, w) != -1)
				{
					if (inPt_cloud[i](2) < inPt_cloud[(visiblityMap)(h, w)](2))
					{
						(visiblityMap)(h, w) = i;
					}
				}
				else
				{
					(visiblityMap)(h, w) = i;
				}
			}
		}
	}
	for (int w = 0; w < this->_image_width; w++)
	{
		for (int h = 0; h < this->_image_height; h++)
		{
			if ((visiblityMap)(h, w) != -1)
			{
				int index = (visiblityMap)(h, w);
				if (isnan(inPt_normals[index](0)))
				{
					std::cout << "there exists a nan normal from warpfeild, skip this one" << std::endl;
					continue;
				}
				if (isnan(in_transMatVec[index](0, 0)))
				{
					std::cout << "there exists a nan transMat from warpfeild, skip this one" << std::endl;
					continue;
				}
				Eigen::Vector3f new_picPt;
				new_picPt(0) = w;
				new_picPt(1) = h;
				new_picPt(2) = 1.0;
				Eigen::Vector3f newpt = this->_depth[h*this->_image_width + w]
					* this->_cameraIntr.inverse()*new_picPt;
				Eigen::Vector3f newInPt;
				TransformPt(inPt_cloud[index], in_transMatVec[index], newInPt);
				Eigen::Vector3f disVec =  newInPt - newpt;
				if (disVec.norm() > 58)
				{
					std::cout << "skip this correspondence" << endl;
					continue;
				}
				corPt_cloud.push_back(newpt);
				outPt_normals.push_back(inPt_normals[index]);
				
				this->_warpField_indexVec.push_back(index);
				outPt_cloud.push_back(inPt_cloud[index]);
				if (isnan(in_transMatVec[index](0, 0)))
				{
					std::cout << "there is nan transmat in input transMatVec while building new warpfeild" << std::endl;
				}
				transMatVec.push_back(in_transMatVec[index]);
				weightVec.push_back(inweightVec[index]);
				
			}
		}
	}
}

void NewtonGuassSolver::FindCorrespondenceCanonical(const float3Vec &inPt_cloud, 
	const float3Vec &inPt_normals,
	float3Vec &outPt_cloud, float3Vec &outPt_normals,
	float3Vec &corPt_cloud, float radius)
{
	MatrixXi visiblityMap(this->_image_height, this->_image_width);
	//this->_visiblityMap = MatrixXi(this->_image_height,this->_image_width);
	visiblityMap.setConstant(-1);
	for (int i = 0; i < inPt_cloud.size(); ++i)
	{
		if (isnan(inPt_cloud[i](0)))
		{
			continue;
		}
		Eigen::Vector3f newPt;
		Matrix4f transMat;
		this->_wf->InterpolateSE3(inPt_cloud[i], transMat, 5);
		TransformPt(inPt_cloud[i], transMat, newPt);
		Eigen::Vector3f picPt = this->_cameraIntr* newPt;
		if (picPt(2) != 0.0)
		{
			int w = int((picPt(0) / float(picPt(2))) + 0.5);
			int h = int((picPt(1) / float(picPt(2))) + 0.5);
			if (h >= this->_image_height || h < 0)
			{
				continue;
			}
			if (w >= this->_image_width || w < 0)
			{
				continue;
			}
			float delt_depth = this->_depth[h* this->_image_width + w] - inPt_cloud[i](2);
			if (abs(delt_depth) < this->_visibilityLimit)
			{
				if ((visiblityMap)(h, w) != -1)
				{
					if (inPt_cloud[i](2) < inPt_cloud[(visiblityMap)(h, w)](2))
					{
						(visiblityMap)(h, w) = i;
					}
				}
				else
				{
					(visiblityMap)(h, w) = i;
				}
			}
		}
	}
	for (int w = 0; w < this->_image_width; w++)
	{
		for (int h = 0; h < this->_image_height; h++)
		{
			if ((visiblityMap)(h, w) != -1)
			{
				int index = (visiblityMap)(h, w);
				if (isnan(inPt_normals[index](0)))
				{
					std::cout << "there exists a nan normal from warpfeild, skip this one" << std::endl;
					continue;
				}
				
				Eigen::Vector3f new_picPt;
				new_picPt(0) = w;
				new_picPt(1) = h;
				new_picPt(2) = 1.0;
				Eigen::Vector3f newpt = this->_depth[h*this->_image_width + w]
					* this->_cameraIntr.inverse()*new_picPt;
				corPt_cloud.push_back(newpt);
				outPt_normals.push_back(inPt_normals[index]);
				outPt_cloud.push_back(inPt_cloud[index]);
			}
		}
	}
}
//double NewtonGuassSolver::CalDataSingleTerm(const Eigen::Vector3f &inpt,
//	const Eigen::Vector3f &normal,
//	const Eigen::Matrix4f &transMat,
//	const Eigen::Vector3f &corpt)
//{
//	Matrix3f rotationMat = transMat.block(0, 0, 3, 3);
//	Vector3f translationMat = transMat.block(0, 3, 3, 1);
//	Eigen::Vector3f newPt = rotationMat* inpt + translationMat;
//	////******** not sure
//	Eigen::Vector3f newNormal = rotationMat* normal;// +translationMat;
//													//newNormal.normalized();
//	double loss = (newNormal.transpose() * (newPt - corpt));
//	if (isnan(transMat(0, 0)))
//	{
//		std::cout << "this point causing nan!! with transMat   " << std::endl;
//	}
//	return 0.01*loss;
//}
double NewtonGuassSolver::CalDataSingleTerm(const Eigen::Vector3f &inpt,
	const Eigen::Vector3f &normal,
	const Eigen::Matrix4f &transMat,
	const Eigen::Vector3f &corpt)
{
	//Matrix3f rotationMat = transMat.block(0, 0, 3, 3);
	//Vector3f translationMat = transMat.block(0, 3, 3, 1);
	//outPt = rotationMat* inPt + translationMat;

	Matrix3f rotationMat = transMat.block(0, 0, 3, 3);
	Vector3f translationMat = transMat.block(0, 3, 3, 1);
	Eigen::Vector3f newPt = rotationMat* inpt + translationMat;
	////******** not sure
	Eigen::Vector3f newNormal = rotationMat* normal;// +translationMat;
	//newNormal.normalized();
	double loss = (newNormal.transpose() * (newPt - corpt));
	if (isnan(transMat(0, 0)))
	{
		std::cout << "this point causing nan!! with transMat   " << std::endl;
	}
	//loss = loss* loss;
	if (isnan(loss))
	{
		std::cout << "this point causing nan!!  pt " << newPt(0) << " " << newPt(1) << " " << newPt(2) << std::endl;
		std::cout << "this point causing nan!!  normal" << normal(0) << " " << normal(1) << " " << normal(2) << std::endl;
		std::cout << "start of matrix" << std::endl;
		std::cout << transMat(0, 0) << ' ' << transMat(0, 1) << ' ' << transMat(0, 2) << ' ' << transMat(0, 3) << std::endl;
		std::cout << transMat(1, 0) << ' ' << transMat(1, 1) << ' ' << transMat(1, 2) << ' ' << transMat(1, 3) << std::endl;
		std::cout << transMat(2, 0) << ' ' << transMat(2, 1) << ' ' << transMat(2, 2) << ' ' << transMat(2, 3) << std::endl;
		std::cout << transMat(3, 0) << ' ' << transMat(3, 1) << ' ' << transMat(3, 2) << ' ' << transMat(3, 3) << std::endl;
		std::cout << "end of matrix" << std::endl;
		std::cout << "this point causing nan!!  Newnormal" << newNormal(0) << " " << newNormal(1) << " " << newNormal(2) << std::endl;
	}
	return 0.01*loss;
}
void NewtonGuassSolver::CalDerivativeNumerical(const Eigen::Vector3f &inpt,
	const Eigen::Vector3f &normal,
	const Eigen::Matrix4f &transMat,
	const Eigen::Vector3f &corpt,
	Eigen::Matrix<double, 6, 1> &derivativeVec)
{

	Eigen::Vector3f newinpt = inpt / float(1000);
	Eigen::Vector3f newcorpt = corpt / float(1000);
	//transMat.block(0, 3, 3, 1) = transMat.block(0, 3, 3, 1) / float(1000);

	if (isnan(inpt(0)))
	{
		std::cout << "there is a nan pt to cal derivative " << inpt(0) << " " << inpt(1) << " " << inpt(2) << std::endl;
	}
	if (isnan(transMat(0,0)))
	{
		std::cout << "there is a nan transMat to cal derivative "  << std::endl;
	}
	Eigen::Matrix4f newTransMat = transMat;
	newTransMat.block(0, 3, 3, 1) = newTransMat.block(0, 3, 3, 1) / float(1000);
	//double originalValue = this->CalDataSingleTerm(newinpt, normal, transMat, corpt);
	double delt = 0.001;
	

	for (int i = 0; i < 6; i++)
	{
		
		Eigen::Matrix<double, 6, 1> ZeroVec0;
		ZeroVec0.setZero();
		ZeroVec0(i, 0) =  -delt;
		Se3<double> deltSe3Neg(ZeroVec0);
		Eigen::Matrix4d deltTransMat0 = deltSe3Neg.GetTransMat4dFromSe3();
		if (isnan(deltTransMat0(0, 0)))
		{
			std::cout << " get a nan transMat from se3" << std::endl;
		}
		Eigen::Matrix4f new_transMat0 = deltTransMat0.cast<float>()*newTransMat;
		double negValue = this->CalDataSingleTerm(newinpt, normal, new_transMat0, newcorpt);
		
		Eigen::Matrix<double, 6, 1> ZeroVec;
		ZeroVec.setZero();
		ZeroVec(i, 0) = delt;
		Se3<double> deltSe3(ZeroVec);
		Eigen::Matrix4d deltTransMat = deltSe3.GetTransMat4dFromSe3();
		Eigen::Matrix4f new_transMat = deltTransMat.cast<float>()*newTransMat;
		if (isnan(inpt(0)))
		{
			std::cout << "there is a nan pt to cal derivative " << inpt(0) << " " << inpt(1) << " " << inpt(2) << std::endl;
		}
		double newValue = this->CalDataSingleTerm(newinpt, normal, new_transMat, newcorpt);
		derivativeVec(i, 0) = (newValue - negValue) / float(delt*2);
	}
}

void NewtonGuassSolver::CalDerivativeNumericalNew(const Eigen::Vector3f &inpt,
	const Eigen::Vector3f &normal,
	const int wfIndex, float radius,
	const Eigen::Vector3f &corpt,
	Eigen::Matrix<double, 6, 1> &derivativeVec)
{
	//Eigen::Vector3f newinpt = inpt / float(1000);
	//Eigen::Vector3f newcorpt = corpt / float(1000);
	if (isnan(inpt(0)))
	{
		std::cout << "there is a nan pt to cal derivative " << inpt(0) << " " << inpt(1) << " " << inpt(2) << std::endl;
	}
	//Eigen::Matrix4f newTransMat = transMat;
	//newTransMat.block(0, 3, 3, 1) = newTransMat.block(0, 3, 3, 1) / float(1000);
	//double originalValue = this->CalDataSingleTerm(newinpt, normal, transMat, corpt);
	double delt = 0.001;
	for (int i = 0; i < 6; i++)
	{
		Eigen::Matrix<double, 6, 1> ZeroVec0;
		//ZeroVec0.setZero();
		//ZeroVec0(i, 0) = -delt;
		/*Se3<double> deltSe3Neg(ZeroVec0);
		Eigen::Matrix4d deltTransMat0 = deltSe3Neg.GetTransMat4dFromSe3();
		if (isnan(deltTransMat0(0, 0)))
		{
			std::cout << " get a nan transMat from se3" << std::endl;
		}*/
		
		//this->_wf->_transMatVec[wfIndex] = deltTransMat0.cast<float>()*this->_wf->_transMatVec[wfIndex];
		Eigen::Matrix4f new_transMat;
		this->_wf->InterpolateSE3(inpt, new_transMat, 5);
		double negValue = this->CalDataSingleTerm(inpt, normal, new_transMat, corpt);
		//this->_wf->_transMatVec[wfIndex] = (deltTransMat0.inverse()).cast<float>()*this->_wf->_transMatVec[wfIndex];
		
		Eigen::Matrix<double, 6, 1> ZeroVec;
		ZeroVec.setZero();
		ZeroVec(i, 0) = delt;
		Se3<double> deltSe3(ZeroVec);
		Eigen::Matrix4d deltTransMat = deltSe3.GetTransMat4dFromSe3();
		this->_wf->_transMatVec[wfIndex] = deltTransMat.cast<float>()*this->_wf->_transMatVec[wfIndex];
		Eigen::Matrix4f new_transMat1; 
		this->_wf->InterpolateSE3(inpt, new_transMat1, 5);
		double newValue = this->CalDataSingleTerm(inpt, normal, new_transMat1,corpt);
		this->_wf->_transMatVec[wfIndex] = (deltTransMat.inverse()).cast<float>()*this->_wf->_transMatVec[wfIndex];
		
		derivativeVec(i, 0) = (newValue - negValue) / double(delt );
	}
}

void NewtonGuassSolver::CalculateDataEnergy(const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const std::vector< Eigen::Matrix4f> &transMatVec,
	const float3Vec &corPt_cloud, VectorXd &dataEngergyMat)
{
	//dataEngergyMat = new MatrixXd(pt_cloud.size(), 1);
	for (int i = 0; i < pt_cloud.size(); i++)
	{
		if (isnan(pt_cloud[i](0)))
		{
			std::cout << "there is a nan normal in normals" << std::endl;
		}
		if (isnan(pt_normals[i](0)))
		{
			std::cout << "there is a nan normal in normals" << std::endl;
		}
		//Eigen::Matrix4f transMat;
		//this->_wf->InterpolateSE3(pt_cloud[i], transMat, 60);
		(dataEngergyMat)(i) = double(
			this->CalDataSingleTerm(pt_cloud[i], pt_normals[i], transMatVec[i], corPt_cloud[i]));
	}
}
//cal all data association energy 
void NewtonGuassSolver::CalculateDataEnergyNew(const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const float3Vec &corPt_cloud, VectorXd &dataEngergyMat, float radius)
{
	//dataEngergyMat = new MatrixXd(pt_cloud.size(), 1);
	for (int i = 0; i < pt_cloud.size(); i++)
	{
		if (isnan(pt_cloud[i](0)))
		{
			std::cout << "there is a nan normal in normals" << std::endl;
		}
		if (isnan(pt_normals[i](0)))
		{
			std::cout << "there is a nan normal in normals" << std::endl;
		}
		Eigen::Matrix4f transMat;
		this->_wf->InterpolateSE3(pt_cloud[i], transMat, radius);
		(dataEngergyMat)(i) = double(
			this->CalDataSingleTerm(pt_cloud[i], pt_normals[i], transMat, corPt_cloud[i]));
	}
}
void NewtonGuassSolver::BuildNewWarpfield(const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const std::vector<float> &weightVec,
	const std::vector< Eigen::Matrix4f> &transMatVec, float dgw)
{
	this->_wf = new Warpfield(pt_cloud, pt_normals, dgw);
	this->_wf->_transMatVec = transMatVec;
	this->_wf->_weightsVec = weightVec;
	//this->_wf->BuildKdTree();
}

void NewtonGuassSolver::BuildDeformationTree(float epsion)
{
	this->_defromationTree = new Warpfield(this->_wf->_vertices, this->_wf->_normals, epsion);
	int size = this->_defromationTree->_vertices.size();
	for (int i = 0; i < size; i++)
	{
		int index = this->_wf->GetNearPtindex(this->_defromationTree->_vertices[i]);
		this->_defromationTree->_transMatVec[i] =(this->_wf->_transMatVec[index]);
		this->_defromationTree->_weightsVec[i] = this->_wf->_weightsVec[index];
	}
	//this->_defromationTree->BuildKdTree();
}

void NewtonGuassSolver::CalculateJacobiMatDataTerm(const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const std::vector< Eigen::Matrix4f> &transMatVec, const float3Vec &corPt_cloud,
	MatrixXd &jacobiMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	//jacobiMat = new MatrixXd(rows, cols);
	jacobiMat.setZero();
	for (int i = 0; i < rows; i++)
	{
		if (isnan(pt_cloud[i](0)))
		{
			std::cout << "there is a nan normal in normals while calculating jacobi data term" << std::endl;
		}
		Eigen::Matrix<double, 1, 6> derivativeVec;
		this->CalculateJacobiDataTermAnalytic(pt_cloud[i].cast<double>(), 
			pt_normals[i].cast<double>(), transMatVec[i].cast<double>(), 
			corPt_cloud[i].cast<double>(), derivativeVec);
		//this->CalDerivativeNumerical(pt_cloud[i], pt_normals[i], transMatVec[i], corPt_cloud[i], derivativeVec);
		for (int j = 0; j < 6; j++)
		{
			(jacobiMat)(i, i * 6 + j) = derivativeVec(0, j);
		}
	}
}
void NewtonGuassSolver::CalculateJacobiMatDataTerm(const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const std::vector< Eigen::Matrix4f> &transMatVec, const float3Vec &corPt_cloud,
	SparseMatrix<double> &jacobiMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	//jacobiMat = new MatrixXd(rows, cols);
	jacobiMat.setZero();
	for (int i = 0; i < rows; i++)
	{
		Eigen::Matrix<double, 1, 6> derivativeVec;
		this->CalculateJacobiDataTermAnalytic(pt_cloud[i].cast<double>(),
			pt_normals[i].cast<double>(), transMatVec[i].cast<double>(),
			corPt_cloud[i].cast<double>(), derivativeVec);
		//Eigen::Matrix<double, 6, 1> derivativeVec;
		//this->CalDerivativeNumerical(pt_cloud[i], pt_normals[i], transMatVec[i], corPt_cloud[i], derivativeVec);
		for (int j = 0; j < 6; j++)
		{
			(jacobiMat).coeffRef(i, i * 6 + j) = derivativeVec(0, j);
		}
	}
}
void NewtonGuassSolver::CalculateJacobiMatDataTermNumerical(const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const std::vector< Eigen::Matrix4f> &transMatVec, const float3Vec &corPt_cloud,
	SparseMatrix<double> &jacobiMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	//jacobiMat = new MatrixXd(rows, cols);
	jacobiMat.setZero();
	for (int i = 0; i < rows; i++)
	{
	
		Eigen::Matrix<double, 6, 1> derivativeVec;
		this->CalDerivativeNumerical(pt_cloud[i], pt_normals[i], transMatVec[i], corPt_cloud[i], derivativeVec);
		for (int j = 0; j < 6; j++)
		{
			(jacobiMat).coeffRef(i, i * 6 + j) = derivativeVec(j, 0);
		}
	}
}
//cal a m* 6n Jacobi matrix
//m is the pt number, n is the warpfield pt number
void NewtonGuassSolver::CalculateJacobiMatDataTermNumericalNew(
	const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const float3Vec &corPt_cloud, float radius,
	SparseMatrix<double> &jacobiMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	jacobiMat.setZero();
	for (int i = 0; i < rows; i++)
	{
		std::vector<int> indexKnnVec;
		std::vector<float> distanceKnnVec;
		pcl::PointXYZ searchPt(pt_cloud[i](0), pt_cloud[i](1), pt_cloud[i](2));
		this->_wf->_kdtree.nearestKSearch(searchPt, 5, indexKnnVec, distanceKnnVec);
		int knn = indexKnnVec.size();
		for (int k = 0; k < knn; k++)
		{
			/*if (distanceKnnVec[k] > 70)
			{
				continue;
			}*/
			Eigen::Matrix<double, 6, 1> derivativeVec;
			this->CalDerivativeNumericalNew(pt_cloud[i], pt_normals[i],
				indexKnnVec[k], radius,corPt_cloud[i], derivativeVec);
			for (int j = 0; j < 6; j++)
			{
				(jacobiMat).coeffRef(i, indexKnnVec[k] * 6 + j) = derivativeVec(j, 0);
			}
		}
	}
}
//cal a m* 6n Jacobi matrix analytic
//m is the pt number, n is the warpfield pt number
void NewtonGuassSolver::CalculateJacobiMatDataTermAnalyticalNew(
	const float3Vec &pt_cloud, const float3Vec &pt_normals,
	const float3Vec &corPt_cloud, float radius,
	SparseMatrix<double> &jacobiMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	jacobiMat.setZero();
	for (int i = 0; i < rows; i++)
	{
		std::vector<int> indexKnnVec;
		std::vector<float> distanceKnnVec;
		pcl::PointXYZ searchPt(pt_cloud[i](0), pt_cloud[i](1), pt_cloud[i](2));
		this->_wf->_kdtree.radiusSearch(searchPt, radius, indexKnnVec, distanceKnnVec);
		if (indexKnnVec.size() == 0)
		{
			this->_wf->_kdtree.nearestKSearch(searchPt, 1, indexKnnVec, distanceKnnVec);
		}
		int index = indexKnnVec[0];
		Eigen::Matrix<double, 1, 6> derivativeVec;
		this->CalculateJacobiDataTermAnalyticNew
			(pt_cloud[i].cast<double>(), pt_normals[i].cast<double>(),this->_wf->_transMatVec[index].cast<double>(),
			 corPt_cloud[i].cast<double>(), derivativeVec);
		for (int j = 0; j < 6; j++)
		{
			(jacobiMat).coeffRef(i, index* 6 + j) = derivativeVec(0, j);
		}
		
	}
}
double NewtonGuassSolver::CalRegPenaltySingleTerm(const Eigen::Vector3f &inpt, 
	const Eigen::Matrix4f &transMat, float weight)
{
	pcl::PointXYZ newPt(inpt(0), inpt(1), inpt(2));
	std::vector<int> ptindex;
	std::vector<float> distanceVec;
	this->_defromationTree->_kdtree.nearestKSearch(newPt, 5, ptindex, distanceVec);
	float sum = 0.0;
	float fi = 0.0000;
	for (int i = 0; i < 5; i++)
	{
		
		Eigen::Vector3f canonical_Pt;
		TransformPt(inpt, transMat, canonical_Pt);
		Eigen::Vector3f canonical_Pt_edge;
		TransformPt(inpt, this->_defromationTree->_transMatVec[ptindex[i]], canonical_Pt_edge);
		Eigen::Vector3f pt = canonical_Pt - canonical_Pt_edge;
		float alpha =Max(this->_defromationTree->_weightsVec[ptindex[i]], weight);
		alpha = alpha;
		sum += fi* alpha* pt.norm();
	}
	if (sum > 1000)
	{
		int a = 0;
	}
	return double(sum);
}

void NewtonGuassSolver::CalRegularizationPenalty(const float3Vec &pt_cloud,
	const std::vector< Eigen::Matrix4f> &transMatVec,
	const std::vector<float> &weightVec,
	VectorXd &dataEngergyMat)
{
	int rows = pt_cloud.size();
	//dataEngergyMat = new MatrixXd(rows, 1);
	dataEngergyMat.setZero();
	for (int i = 0; i < rows; i++)
	{
		(dataEngergyMat)(i, 0) = this->CalRegPenaltySingleTerm(pt_cloud[i], transMatVec[i], weightVec[i]);
	}
}

void NewtonGuassSolver::CalJacobiMatReg(const float3Vec &pt_cloud,
	const std::vector< Eigen::Matrix4f> &transMatVec,
	const std::vector<float> &weightVec, MatrixXd &jacobiRegMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	double delt = 0.0001;
	//jacobiRegMat = new MatrixXd(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		double origin_value = this->CalRegPenaltySingleTerm(pt_cloud[i], transMatVec[i], weightVec[i]);
		for (int j = 0; j < 6; j++)
		{
			
			Eigen::Matrix<double, 6, 1> ZeroVec0;
			ZeroVec0.setZero();
			//ZeroVec0(j, 0) = -delt;
			//Se3<double> deltSe3Neg(ZeroVec0);
			//Eigen::Matrix4d deltTransMat0 = deltSe3Neg.GetTransMat4dFromSe3();
			//Eigen::Matrix4f new_transMat0 = deltTransMat0.cast<float>()*transMatVec[i];
			double neg_value = this->CalRegPenaltySingleTerm(pt_cloud[i], transMatVec[i], weightVec[i]);
			
			Eigen::Matrix<double, 6, 1> ZeroVec;
			ZeroVec.setZero();
			ZeroVec(j, 0) = delt;
			Se3<double> deltSe3(ZeroVec);
			Eigen::Matrix4d deltTransMat = deltSe3.GetTransMat4dFromSe3();
			Eigen::Matrix4f new_transMat = deltTransMat.cast<float>()*transMatVec[i];
			double new_value = this->CalRegPenaltySingleTerm(pt_cloud[i], new_transMat, weightVec[i]);
			///
			if (new_value > 1000 || new_value < -1000)
			{
				std::cout << "big numbers" << std::endl;
			}
			(jacobiRegMat)(i, i * 6 + j) = (new_value- neg_value)/(delt);
		}
		
		pcl::PointXYZ pt(pt_cloud[i](0), pt_cloud[i](1), pt_cloud[i](2));

		std::vector<int> indexVec;
		std::vector<float> distanceVec;
		int pt_index = this->_wf->GetNearPtindex(pt_cloud[i]);
		this->_defromationTree->_kdtree.nearestKSearch(pt, 5, indexVec, distanceVec);
		for (int j = 0; j < 5; j++)
		{
			int index = this->_wf->GetNearPtindex(this->_defromationTree->_vertices[indexVec[j]]);
			if (index == pt_index)
			{
				continue;
			}
			for (int k = 0; k < 6; k++)
			{
				Eigen::Matrix<double, 6, 1> ZeroVec;
				ZeroVec.setZero();
				ZeroVec(j, 0) = delt;
				Se3<double> deltSe3(ZeroVec);
				Eigen::Matrix4d deltTransMat = deltSe3.GetTransMat4dFromSe3();
				Eigen::Matrix4f new_transMat = deltTransMat.cast<float>()*transMatVec[index];
				double new_value = this->CalRegPenaltySingleTerm(pt_cloud[i], new_transMat, weightVec[i]);
				if (new_value > 1000 || new_value < -1000)
				{
					std::cout << "big numbers" << std::endl;
				}
				(jacobiRegMat)(i, index * 6 + k) = (new_value - origin_value) / delt;
			 }
			
		}
		
	}
}
void NewtonGuassSolver::CalJacobiMatReg(const float3Vec &pt_cloud,
	const std::vector< Eigen::Matrix4f> &transMatVec,
	const std::vector<float> &weightVec, SparseMatrix<double> &jacobiRegMat)
{
	int rows = pt_cloud.size();
	int cols = rows * 6;
	double delt = 0.001;
	//jacobiRegMat = new MatrixXd(rows, cols);
	for (int i = 0; i < rows; i++)
	{
		double origin_value = this->CalRegPenaltySingleTerm(pt_cloud[i], transMatVec[i], weightVec[i]);
		for (int j = 0; j < 6; j++)
		{
			Eigen::Matrix<double, 6, 1> ZeroVec;
			ZeroVec.setZero();
			ZeroVec(j, 0) = delt;
			Se3<double> deltSe3(ZeroVec);
			Eigen::Matrix4d deltTransMat = deltSe3.GetTransMat4dFromSe3();
			Eigen::Matrix4f new_transMat = deltTransMat.cast<float>()*transMatVec[i];
			double new_value = this->CalRegPenaltySingleTerm(pt_cloud[i], new_transMat, weightVec[i]);
			///
			if (new_value > 1000 || new_value < -1000)
			{
				std::cout << "big numbers" << std::endl;
			}
			(jacobiRegMat).coeffRef(i, i * 6 + j) = (new_value - origin_value) / delt;
		}
		
		pcl::PointXYZ pt(pt_cloud[i](0), pt_cloud[i](1), pt_cloud[i](2));

		std::vector<int> indexVec;
		std::vector<float> distanceVec;
		int pt_index = this->_wf->GetNearPtindex(pt_cloud[i]);
		this->_defromationTree->_kdtree.nearestKSearch(pt, 5, indexVec, distanceVec);
		for (int j = 0; j < 5; j++)
		{
			int index = this->_wf->GetNearPtindex(this->_defromationTree->_vertices[indexVec[j]]);
			if (index == pt_index)
			{
			continue;
			}
			for (int k = 0; k < 6; k++)
			{
				Eigen::Matrix<double, 6, 1> ZeroVec;
				ZeroVec.setZero();
				ZeroVec(j, 0) = delt;
				Se3<double> deltSe3(ZeroVec);
				Eigen::Matrix4d deltTransMat = deltSe3.GetTransMat4dFromSe3();
				Eigen::Matrix4f new_transMat = deltTransMat.cast<float>()*transMatVec[index];
				double new_value = this->CalRegPenaltySingleTerm(pt_cloud[i], new_transMat, weightVec[i]);
				if (new_value > 1000 || new_value < -1000)
				{
					std::cout << "big numbers" << std::endl;
				}
				(jacobiRegMat).coeffRef(i, index * 6 + k) = (new_value - origin_value) / delt;
			}
		}
	}
}
/*
void NewtonGuassSolver::NewtonGaussOptimizer(Warpfield *wf)
{
	FileIO fio;
	float epsion = 100;
	float3Vec outPt_cloud;
	float3Vec outPt_normals;
	std::vector<float> weightVec;
	std::vector< Eigen::Matrix4f> transMatVec;
	float3Vec corPt_cloud;
	FindCorrespondence(wf->_vertices,wf->_normals,wf->_weightsVec,wf->_transMatVec,
		outPt_cloud, outPt_normals,weightVec,transMatVec,corPt_cloud);

	BuildNewWarpfield(outPt_cloud, outPt_normals, weightVec, transMatVec);
	BuildDeformationTree(epsion);

	VectorXd dataEngergyMat(outPt_cloud.size());
	dataEngergyMat.setZero();
	CalculateDataEnergy(outPt_cloud, outPt_normals,transMatVec,
		corPt_cloud, dataEngergyMat);
	MatrixXd jacobiMatData(outPt_cloud.size(),6* outPt_cloud.size());
	jacobiMatData.setZero();
	CalculateJacobiMatDataTerm(outPt_cloud, outPt_normals,
		transMatVec, corPt_cloud,jacobiMatData);

	MatrixXd hessianMatData(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	hessianMatData = (jacobiMatData).transpose()*(jacobiMatData);

	VectorXd regEngergyMat(outPt_cloud.size());
	regEngergyMat.setZero();
	CalRegularizationPenalty(outPt_cloud,transMatVec,weightVec, regEngergyMat);

	MatrixXd jacobiRegMat(outPt_cloud.size(),6* outPt_cloud.size());
	jacobiRegMat.setZero();
	CalJacobiMatReg(outPt_cloud,transMatVec,weightVec, jacobiRegMat);

	MatrixXd hessianMatReg(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	hessianMatReg.setZero();
	MatrixXd jacobiRegMatTrans = (jacobiRegMat).transpose();
	
	hessianMatReg = jacobiRegMatTrans * (jacobiRegMat);
	fio.saveMatrix(hessianMatReg, "E:\\kinfu\\data\\mat\\hessianMatReg.txt");
	MatrixXd jacobiMat(outPt_cloud.size(), 6 * outPt_cloud.size());
	jacobiMat.setZero();
	jacobiMat = jacobiMatData + 200 * (jacobiRegMat);
	fio.saveMatrix(jacobiMatData, "E:\\kinfu\\data\\mat\\jacobiMatData.txt");
	fio.saveMatrix(jacobiRegMat, "E:\\kinfu\\data\\mat\\jacobiRegMat.txt");
	MatrixXd hessianMat(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	hessianMat = hessianMatData + 200 * hessianMatReg;
	fio.saveMatrix(hessianMat, "E:\\kinfu\\data\\mat\\hessianMat.txt");
	VectorXd energyMat(outPt_cloud.size());
	energyMat = (dataEngergyMat) + 200 * (regEngergyMat);
	fio.saveMatrix(energyMat, "E:\\kinfu\\data\\mat\\energyMat.txt");

	Eigen::SparseMatrix<double> hessianMatSparse(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	for (int i = 0; i < 6 * outPt_cloud.size(); i++)
	{
		for (int j = 0; j < 6 * outPt_cloud.size(); j++)
		{
			hessianMatSparse.coeffRef(i,j) = hessianMat(i, j);
		}
	}
	//hessianMatSparse;
	//(hessianMat);

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMatSparse);
	Eigen::LDLT<MatrixXd> cholesky(hessianMat);

	MatrixXd jacobiMatTrans = jacobiMat.transpose();
	fio.saveMatrix(jacobiMatTrans, "E:\\kinfu\\data\\mat\\jacobiMatTrans.txt");
	VectorXd B(6 * outPt_cloud.size());
	B.setZero();
	for (int i = 0; i < outPt_cloud.size(); i++)
	{
		for (int j = 0; j < 6; j++)
		{
			B(i * 6 + j) = jacobiMatTrans(i * 6 + j, i) * energyMat(i);
		}
	}
	//B = jacobiMatTrans * (energyMat);
	fio.saveMatrix(B, "E:\\kinfu\\data\\mat\\B.txt");
	VectorXd deltX = cholesky.solve(B);
	VectorXd deltX2 = choleskeySparse.solve(B);
	fio.saveMatrix(deltX2, "E:\\kinfu\\data\\mat\\deltX2.txt");
	bool a_solution_exists = (hessianMat*deltX).isApprox(B, 0.01);
	//if (a_solution_exists)
	//{
	int size = outPt_cloud.size();
	for (int i = 0; i < size; i++)
	{
		MatrixXd se3value(6, 1);
		for (int j = 0; j < 6; j++)
		{
			double value = deltX2(i * 6 + j);
			se3value(j, 0) = value;
		}
		Se3<double> deltse3(se3value);
		Matrix4f deltTrans = deltse3.GetTransMat4dFromSe3().cast<float>();
		this->_wf->_transMatVec[i] = deltTrans * this->_wf->_transMatVec[i];
	}
	//}
	int wfsize = wf->_vertices.size();
	for (int i = 0; i < wfsize; i++)
	{
		this->_wf->InterpolateSE3(wf->_vertices[i], wf->_transMatVec[i], 5);
	}
	delete this->_wf;
	delete this->_defromationTree;
}
*/
void NewtonGuassSolver::NGOptimizer(Warpfield **wf, int iterationCount,
	float3Vec &corPt_cloud)
{
	FileIO fio;
	
	std::cout << "NewWarpfield  point numbers" << this->_wf->_vertices.size() << std::endl;
	VectorXd dataEngergyMat(this->_wf->_vertices.size());
	dataEngergyMat.setZero();
	CalculateDataEnergy(this->_wf->_vertices, this->_wf->_normals, this->_wf->_transMatVec,
		corPt_cloud, dataEngergyMat);
	fio.saveMatrix(dataEngergyMat, "E:\\kinfu\\data\\mat\\dataEngergyMat" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double> jacobiMatData(this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	jacobiMatData.setZero();
	CalculateJacobiMatDataTermNumerical(this->_wf->_vertices, this->_wf->_normals,
		this->_wf->_transMatVec, corPt_cloud, jacobiMatData);
	fio.saveMatrix(jacobiMatData, "E:\\kinfu\\data\\mat\\jacobiMatData" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double>  hessianMatData(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMatData = (jacobiMatData).transpose()*(jacobiMatData);

	VectorXd regEngergyMat(this->_wf->_vertices.size());
	regEngergyMat.setZero();
	CalRegularizationPenalty(this->_wf->_vertices, this->_wf->_transMatVec, this->_wf->_weightsVec, regEngergyMat);
	fio.saveMatrix(regEngergyMat, "E:\\kinfu\\data\\mat\\regEngergyMat" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double>  jacobiRegMat(this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	jacobiRegMat.setZero();
	CalJacobiMatReg(this->_wf->_vertices, this->_wf->_transMatVec, this->_wf->_weightsVec, jacobiRegMat);

	SparseMatrix<double>  hessianMatReg(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMatReg.setZero();
	SparseMatrix<double>  jacobiRegMatTrans = (jacobiRegMat).transpose();
	hessianMatReg = jacobiRegMatTrans * (jacobiRegMat);

	fio.saveMatrix(hessianMatReg, "E:\\kinfu\\data\\mat\\hessianMatReg" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double> jacobiMat(this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	jacobiMat.setZero();
	jacobiMat = jacobiMatData + 200 * (jacobiRegMat);
	fio.saveMatrix(jacobiRegMat, "E:\\kinfu\\data\\mat\\jacobiRegMat" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double> hessianMat(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMat = hessianMatData + 200 * hessianMatReg;
	fio.saveMatrix(hessianMat, "E:\\kinfu\\data\\mat\\hessianMat" + std::to_string(iterationCount) + ".txt");
	VectorXd energyMat(this->_wf->_vertices.size());
	energyMat = (dataEngergyMat)+200 * (regEngergyMat);
	fio.saveMatrix(energyMat, "E:\\kinfu\\data\\mat\\energyMat" + std::to_string(iterationCount) + ".txt");

	//hessianMatSparse;
	//(hessianMat);

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMat);

	SparseMatrix<double> jacobiMatTrans = jacobiMat.transpose();
	fio.saveMatrix(jacobiMatTrans, "E:\\kinfu\\data\\mat\\jacobiMatTrans" + std::to_string(iterationCount) + ".txt");
	VectorXd B(6 * this->_wf->_vertices.size());
	B.setZero();
	
	B = jacobiMatTrans *(energyMat);
	fio.saveMatrix(B, "E:\\kinfu\\data\\mat\\B" + std::to_string(iterationCount) + ".txt");
	//VectorXd deltX = cholesky.solve(B);
	VectorXd deltX2 = B;//choleskeySparse.solve(B);
	fio.saveMatrix(deltX2, "E:\\kinfu\\data\\mat\\deltX2" + std::to_string(iterationCount) + ".txt");
	bool a_solution_exists = (hessianMat*deltX2).isApprox(B, 1);
	std::cout << "cholesky solution " << a_solution_exists << std::endl;
	//if (a_solution_exists)
	//{
	int size = this->_wf->_vertices.size();
	for (int i = 0; i < size; i++)
	{
		MatrixXd se3value(6, 1);
		for (int j = 0; j < 6; j++)
		{
			//std::cout << "step 0.00005 " << a_solution_exists << std::endl;
			double value = -deltX2(i * 6 + j);
			se3value(j, 0) = value;
		}
		Se3<double> deltse3(se3value);

		Matrix4f deltTrans = deltse3.GetTransMat4dFromSe3().cast<float>();
		deltTrans.block(0, 3, 3, 1) = deltTrans.block(0, 3, 3, 1) * 1000;
		Eigen::Vector3f transPt;
		TransformPt(this->_wf->_vertices[i], deltTrans, transPt);
		Eigen::Vector3f transDeltPt = transPt - this->_wf->_vertices[i];
		if (transDeltPt.norm() > 30)
		{
			std::cout << "too big step for this point in warpField" << std::endl;
		}
	
		this->_wf->_transMatVec[i] = deltTrans * this->_wf->_transMatVec[i];
	}
	float3Vec newWfPtcloud;
	int wfsize = (*wf)->_vertices.size();
	std::cout << "the wf point number to be interpolated " << wfsize << std::endl;
	for (int i = 0; i < wfsize; i++)
	{
		this->_wf->InterpolateSE3((*wf)->_vertices[i], (*wf)->_transMatVec[i], 1);
		if (isnan((*wf)->_transMatVec[i](0, 0)))
		{
			std::cout << "update warpfield while there is a point with a nan transMat interpolated" << std::endl;
		}
		Vector3f newPt;
		TransformPt((*wf)->_vertices[i], (*wf)->_transMatVec[i], newPt);
		newWfPtcloud.push_back(newPt);
	}
	string outNgwfPt = "E:\\kinfu\\data\\mat\\outNgwfPt" + std::to_string(iterationCount) + ".ply";
	fio.SavePintsWithNormals(newWfPtcloud, (*wf)->_normals, outNgwfPt);
}
//data association contains all pts generated by marching cubes
void NewtonGuassSolver::NGOptimizerNew(const float3Vec &cananicalPts, 
	const float3Vec &cananicalNormals,
	int iterationCount,float radius,
	const float3Vec &corPt_cloud)
{
	int dataPt_size = cananicalPts.size();
	FileIO fio;
	std::cout << "NewWarpfield  point numbers" << this->_wf->_vertices.size() << std::endl;
	VectorXd dataEngergyMat(dataPt_size);
	dataEngergyMat.setZero();
	CalculateDataEnergyNew(cananicalPts, cananicalNormals, corPt_cloud, dataEngergyMat, radius);
	fio.saveMatrix(dataEngergyMat, "E:\\kinfu\\data\\mat\\dataEngergyMat" + std::to_string(iterationCount) + ".txt");
	std::cout << "          &&&&&!!!!!!!energy loss ************   " << dataEngergyMat.squaredNorm() << std::endl;
	SparseMatrix<double> jacobiMatData(dataPt_size, 6 * this->_wf->_vertices.size());
	jacobiMatData.setZero();
	CalculateJacobiMatDataTermAnalyticalNew(cananicalPts, cananicalNormals,
		corPt_cloud, radius, jacobiMatData);
	fio.saveMatrix(jacobiMatData, "E:\\kinfu\\data\\mat\\jacobiMatData" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double>  hessianMatData(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMatData = (jacobiMatData).transpose()*(jacobiMatData);

	VectorXd regEngergyMat(this->_wf->_vertices.size());
	regEngergyMat.setZero();
	CalRegularizationPenalty(this->_wf->_vertices, this->_wf->_transMatVec, this->_wf->_weightsVec, regEngergyMat);
	fio.saveMatrix(regEngergyMat, "E:\\kinfu\\data\\mat\\regEngergyMat" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double>  jacobiRegMat(this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	jacobiRegMat.setZero();
	CalJacobiMatReg(this->_wf->_vertices, this->_wf->_transMatVec, this->_wf->_weightsVec, jacobiRegMat);

	SparseMatrix<double>  hessianMatReg(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMatReg.setZero();
	SparseMatrix<double>  jacobiRegMatTrans = (jacobiRegMat).transpose();
	hessianMatReg = jacobiRegMatTrans * (jacobiRegMat);
	fio.saveMatrix(hessianMatReg, "E:\\kinfu\\data\\mat\\hessianMatReg" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double> hessianMat(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMat = hessianMatData + 200 * hessianMatReg;
	fio.saveMatrix(hessianMat, "E:\\kinfu\\data\\mat\\hessianMat" + std::to_string(iterationCount) + ".txt");
	

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMat);

	MatrixXd jacobiDataEnergyMat = (jacobiMatData.transpose()) * dataEngergyMat;
	fio.saveMatrix(jacobiDataEnergyMat, "E:\\kinfu\\data\\mat\\jacobiMatTrans" + std::to_string(iterationCount) + ".txt");
	
	MatrixXd jacobiRegEnergyMat = (jacobiRegMatTrans) * regEngergyMat;
	VectorXd B(6 * this->_wf->_vertices.size());
	B.setZero();

	B = jacobiDataEnergyMat + 200 * jacobiRegEnergyMat;
	fio.saveMatrix(B, "E:\\kinfu\\data\\mat\\B" + std::to_string(iterationCount) + ".txt");
	//Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMat);
	VectorXd deltX = choleskeySparse.solve(B);
	//VectorXd deltX2 = B;//choleskeySparse.solve(B);
	fio.saveMatrix(deltX, "E:\\kinfu\\data\\mat\\deltX2" + std::to_string(iterationCount) + ".txt");
	bool a_solution_exists = (hessianMat*deltX).isApprox(B, 1);
	std::cout << "cholesky solution " << a_solution_exists << std::endl;
	
	int size = this->_wf->_vertices.size();
	for (int i = 0; i < size; i++)
	{
		MatrixXd se3value(6, 1);
		for (int j = 0; j < 6; j++)
		{
			double value = -deltX(i * 6 + j);
			se3value(j, 0) = value;
		}
		Se3<double> deltse3(se3value);
		Matrix4f deltTrans = deltse3.GetTransMat4dFromSe3().cast<float>();
		//deltTrans.block(0, 3, 3, 1) = deltTrans.block(0, 3, 3, 1) ;
		Eigen::Vector3f transPt;
		TransformPt(this->_wf->_vertices[i], deltTrans, transPt);
		Eigen::Vector3f transDeltPt = transPt - this->_wf->_vertices[i];
		this->_wf->_transMatVec[i] = deltTrans * this->_wf->_transMatVec[i];
	}
	float3Vec newWfPtcloud;
	int wfsize = (this->_wf)->_vertices.size();
	std::cout << "the wf point number to be interpolated " << wfsize << std::endl;
	for (int i = 0; i < wfsize; i++)
	{
		//this->_wf->InterpolateSE3((this->_wf)->_vertices[i], (this->_wf)->_transMatVec[i], 1);
		if (isnan((this->_wf)->_transMatVec[i](0, 0)))
		{
			std::cout << "update warpfield while there is a point with a nan transMat interpolated" << std::endl;
		}
		Vector3f newPt;
		TransformPt((this->_wf)->_vertices[i], (this->_wf)->_transMatVec[i], newPt);
		newWfPtcloud.push_back(newPt);
	}
	string outNgwfPt = "E:\\kinfu\\data\\mat\\outNgwfPt" + std::to_string(iterationCount) + ".ply";
	fio.SavePintsWithNormals(newWfPtcloud, (this->_wf)->_normals, outNgwfPt);
}
void NewtonGuassSolver::NGOptimizerNewNumerical(const float3Vec &cananicalPts,
	const float3Vec &cananicalNormals,
	int iterationCount, float radius,float rate,
	const float3Vec &corPt_cloud)
{
	int dataPt_size = cananicalPts.size();
	FileIO fio;
	std::cout << "NewWarpfield  point numbers" << this->_wf->_vertices.size() << std::endl;
	VectorXd dataEngergyMat(dataPt_size);
	dataEngergyMat.setZero();
	CalculateDataEnergyNew(cananicalPts, cananicalNormals, corPt_cloud, dataEngergyMat, radius);
	fio.saveMatrix(dataEngergyMat, "E:\\kinfu\\data\\mat\\dataEngergyMat" + std::to_string(iterationCount) + ".txt");
	std::cout << "          &&&&&!!!!!!!energy loss ************   " << dataEngergyMat.squaredNorm() << std::endl;
	SparseMatrix<double> jacobiMatData(dataPt_size, 6 * this->_wf->_vertices.size());
	jacobiMatData.setZero();
	this->CalculateJacobiMatDataTermNumericalNew(cananicalPts, cananicalNormals,
		corPt_cloud, radius, jacobiMatData);
	fio.saveMatrix(jacobiMatData, "E:\\kinfu\\data\\mat\\jacobiMatData" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double>  hessianMatData(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMatData = (jacobiMatData).transpose()*(jacobiMatData);

	VectorXd regEngergyMat(this->_wf->_vertices.size());
	regEngergyMat.setZero();
	CalRegularizationPenalty(this->_wf->_vertices, this->_wf->_transMatVec, this->_wf->_weightsVec, regEngergyMat);
	fio.saveMatrix(regEngergyMat, "E:\\kinfu\\data\\mat\\regEngergyMat" + std::to_string(iterationCount) + ".txt");
	std::cout << "          &&&&&!!!!!!!reg energy loss ************   **" << regEngergyMat.squaredNorm() << std::endl;
	SparseMatrix<double>  jacobiRegMat(this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	jacobiRegMat.setZero();
	CalJacobiMatReg(this->_wf->_vertices, this->_wf->_transMatVec, this->_wf->_weightsVec, jacobiRegMat);

	SparseMatrix<double>  hessianMatReg(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMatReg.setZero();
	SparseMatrix<double>  jacobiRegMatTrans = (jacobiRegMat).transpose();
	hessianMatReg = jacobiRegMatTrans * (jacobiRegMat);
	fio.saveMatrix(hessianMatReg, "E:\\kinfu\\data\\mat\\hessianMatReg" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double> hessianMat(6 * this->_wf->_vertices.size(), 6 * this->_wf->_vertices.size());
	hessianMat = hessianMatData + 200 * hessianMatReg;
	fio.saveMatrix(hessianMat, "E:\\kinfu\\data\\mat\\hessianMat" + std::to_string(iterationCount) + ".txt");

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMat);

	MatrixXd jacobiDataEnergyMat = (jacobiMatData.transpose()) * dataEngergyMat;
	fio.saveMatrix(jacobiDataEnergyMat, "E:\\kinfu\\data\\mat\\jacobiMatTrans" + std::to_string(iterationCount) + ".txt");

	MatrixXd jacobiRegEnergyMat = (jacobiRegMatTrans)* regEngergyMat;
	VectorXd B(6 * this->_wf->_vertices.size());
	B.setZero();
	/*for (int i = 0; i < jacobiDataEnergyMat.size(); i++)
	{
		if (jacobiDataEnergyMat(i) < 0)
		{
			B(i) = jacobiDataEnergyMat(i) + 200 * jacobiRegEnergyMat(i);
		}
		else {
			B(i) = jacobiDataEnergyMat(i) - 200 * jacobiRegEnergyMat(i);
		}
	}*/
	B = jacobiDataEnergyMat + 200 * jacobiRegEnergyMat;
	fio.saveMatrix(B, "E:\\kinfu\\data\\mat\\B" + std::to_string(iterationCount) + ".txt");
	//Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMat);
	VectorXd deltX = choleskeySparse.solve(B);
	//VectorXd deltX2 = B;//choleskeySparse.solve(B);
	fio.saveMatrix(deltX, "E:\\kinfu\\data\\mat\\deltX2" + std::to_string(iterationCount) + ".txt");
	bool a_solution_exists = (hessianMat*deltX).isApprox(B, 1);
	std::cout << "cholesky solution " << a_solution_exists << std::endl;

	int size = this->_wf->_vertices.size();
	for (int i = 0; i < size; i++)
	{
		MatrixXd se3value(6, 1);
		for (int j = 0; j < 6; j++)
		{
			double value = -rate*deltX(i * 6 + j);
			se3value(j, 0) = value;
		}
		Se3<double> deltse3(se3value);
		Matrix4f deltTrans = deltse3.GetTransMat4dFromSe3().cast<float>();
		//deltTrans.block(0, 3, 3, 1) = deltTrans.block(0, 3, 3, 1) ;
		Eigen::Vector3f transPt;
		TransformPt(this->_wf->_vertices[i], deltTrans, transPt);
		Eigen::Vector3f transDeltPt = transPt - this->_wf->_vertices[i];
		this->_wf->_transMatVec[i] = deltTrans * this->_wf->_transMatVec[i];
	}
	float3Vec newWfPtcloud;
	int wfsize = (this->_wf)->_vertices.size();
	std::cout << "the wf point number to be interpolated " << wfsize << std::endl;
	for (int i = 0; i < wfsize; i++)
	{
		//this->_wf->InterpolateSE3((this->_wf)->_vertices[i], (this->_wf)->_transMatVec[i], 1);
		if (isnan((this->_wf)->_transMatVec[i](0, 0)))
		{
			std::cout << "update warpfield while there is a point with a nan transMat interpolated" << std::endl;
		}
		Vector3f newPt;
		TransformPt((this->_wf)->_vertices[i], (this->_wf)->_transMatVec[i], newPt);
		newWfPtcloud.push_back(newPt);
	}
	string outNgwfPt = "E:\\kinfu\\data\\mat\\outNgwfPt" + std::to_string(iterationCount) + ".ply";
	fio.SavePintsWithNormals(newWfPtcloud, (this->_wf)->_normals, outNgwfPt);
}
void NewtonGuassSolver::NGOptimizerTest(Warpfield **wf, int iterationCount,
	float3Vec &corPt_cloud)
{
	double energy = this->CalDataSingleTerm(this->_wf->_vertices[0], this->_wf->_normals[0],
		this->_wf->_transMatVec[0], corPt_cloud[0]);
	std::cout << "****energy 0 " << iterationCount<< "  " <<energy << std::endl;
	Matrix<double, 1, 6> Jacobi;
	this->CalculateJacobiDataTermAnalytic(this->_wf->_vertices[0].cast<double>(), 
		this->_wf->_normals[0].cast<double>(),
		this->_wf->_transMatVec[0].cast<double>(), corPt_cloud[0].cast<double>(), Jacobi);
	
	//Matrix<double, 6, 6> Hessian  = Jacobi.transpose()* Jacobi;
	//Eigen::SimplicialCholesky<Matrix<double,6,6>> choleskey(Hessian);
	//Matrix<double, 6, 6> HessianInv = Hessian.inverse();
	Matrix<double, 6, 1> deltB = Jacobi.transpose() * energy;
	Matrix<double, 6, 1> deltX = deltB;  //Hessian.llt().solve(deltB);
	Se3<double> deltmat(-deltX);
	if (isnan(deltX(0, 0)))
	{
		std::cout << "not good, there is a nan deltX" << std::endl;
		/*std::cout << deltX(0, 0) << ' ' << deltX(1, 0) << ' ' << deltX(2, 0) << ' '
			<< deltX(3,0) << ' '
			<< deltX(4, 0) << ' '
			<< deltX(5, 0) << std::endl;*/
		return;
		
	}
	else
	{
		/*std::cout << "there is a deltX" << std::endl;
		std::cout << deltX(0, 0) << ' ' << deltX(1, 0) << ' ' << deltX(2, 0) << ' '
			<< deltX(3, 0) << ' '
			<< deltX(4, 0) << ' '
			<< deltX(5, 0) << std::endl;*/
	}
	Matrix4d deltTransMat = deltmat.GetTransMat4dFromSe3();
	deltTransMat.block(0, 3, 3, 1) = deltTransMat.block(0, 3, 3, 1) * 1000.0;
	if (isnan(deltTransMat(0, 0)))
	{
		std::cout << "not good, there is a nan transmat" << std::endl;
	}
	std::cout << "SE3 deltTrans start of matrix" << std::endl;
	std::cout << deltTransMat(0, 0) << ' ' << deltTransMat(0, 1) << ' ' << deltTransMat(0, 2) << ' ' << deltTransMat(0, 3) << std::endl;
	std::cout << deltTransMat(1, 0) << ' ' << deltTransMat(1, 1) << ' ' << deltTransMat(1, 2) << ' ' << deltTransMat(1, 3) << std::endl;
	std::cout << deltTransMat(2, 0) << ' ' << deltTransMat(2, 1) << ' ' << deltTransMat(2, 2) << ' ' << deltTransMat(2, 3) << std::endl;
	std::cout << deltTransMat(3, 0) << ' ' << deltTransMat(3, 1) << ' ' << deltTransMat(3, 2) << ' ' << deltTransMat(3, 3) << std::endl;
	std::cout << "end of deltTrans matrix" << std::endl;
	this->_wf->_transMatVec[0] = 
		deltTransMat.cast<float>()*(this->_wf->_transMatVec[0]);
	energy = this->CalDataSingleTerm(this->_wf->_vertices[0], this->_wf->_normals[0],
		this->_wf->_transMatVec[0], corPt_cloud[0]);
	std::cout << "****energy 1 " << iterationCount << "  " << energy << std::endl;
}
void NewtonGuassSolver::NewtonGaussOptimizer2(Warpfield **wf, int iterationCount,
	const float3Vec &mcPts, const float3Vec &mcNormals)
{
	FileIO fio;
	float radius = 50;
	float3Vec outPt_cloud;
	float3Vec outPt_normals;
	std::vector<float> weightVec;
	std::vector< Eigen::Matrix4f> transMatVec;
	float3Vec corPt_cloud;
	FindCorrespondence((*wf)->_vertices, (*wf)->_normals, (*wf)->_weightsVec, (*wf)->_transMatVec,
		outPt_cloud, outPt_normals, weightVec, transMatVec, corPt_cloud);
	std::string corPath = "E:\\kinfu\\data\\mat\\corPt" + std::to_string(iterationCount) + ".ply";

	fio.SavePintsWithNormals(corPt_cloud, outPt_normals, corPath);
	std::string corwarpPath = "E:\\kinfu\\data\\mat\\corwarpPt" + std::to_string(iterationCount) + ".ply";
	fio.SavePintsWithNormals(outPt_cloud, outPt_normals, corwarpPath);
	std::cout << "input warpfield point numbers" << (*wf)->_vertices.size() << std::endl;
	BuildNewWarpfield(outPt_cloud, outPt_normals, weightVec, transMatVec, 25);
	BuildDeformationTree(radius);

	float3Vec cananicalPt_cloud;
	float3Vec cananicalPt_normals;
	float3Vec corLivePt;
	this->FindCorrespondenceCanonical(mcPts, mcNormals, cananicalPt_cloud, 
		cananicalPt_normals, corLivePt, radius);

	//NGOptimizerTest(wf, iterationCount, corPt_cloud);
	//NGOptimizerTest(wf, iterationCount +1, corPt_cloud);
	//NGOptimizerTest(wf, iterationCount +2, corPt_cloud);

	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals, iterationCount, radius, 1.0,corLivePt);
	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals , iterationCount + 20000, radius,1.0, corLivePt);
	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals , iterationCount + 10000, radius,1.0, corLivePt);
	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals, iterationCount + 10500, radius,1.0, corLivePt);
	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals, iterationCount + 11000, radius, 1.0, corLivePt);
	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals, iterationCount + 12000, radius, 1.0, corLivePt);
	NGOptimizerNewNumerical(cananicalPt_cloud, cananicalPt_normals, iterationCount + 13000, radius, 1.0, corLivePt);
	int wfsize = (*wf)->_vertices.size();
	for (int index = 0; index < wfsize; index++)
	{
		this->_wf->InterpolateSE3((*wf)->_vertices[index], (*wf)->_transMatVec[index], 5);
	}
	delete this->_wf;
	delete this->_defromationTree;
}
void NewtonGuassSolver::NewtonGaussOptimizer(Warpfield **wf, int iterationCount)
{
	FileIO fio;
	float epsion = 50;
	float3Vec outPt_cloud;
	float3Vec outPt_normals;
	std::vector<float> weightVec;
	std::vector< Eigen::Matrix4f> transMatVec;
	float3Vec corPt_cloud;
	FindCorrespondence((*wf)->_vertices, (*wf)->_normals, (*wf)->_weightsVec, (*wf)->_transMatVec,
		outPt_cloud, outPt_normals, weightVec, transMatVec, corPt_cloud);
	std::string corPath = "E:\\kinfu\\data\\mat\\corPt" + std::to_string(iterationCount) +".ply";

	fio.SavePintsWithNormals(outPt_cloud, outPt_normals, corPath);
	std::cout << "input warpfield point numbers" << (*wf)->_vertices.size() << std::endl;
	BuildNewWarpfield(outPt_cloud, outPt_normals, weightVec, transMatVec, epsion/2);
	BuildDeformationTree(epsion);
	std::cout << "NewWarpfield  point numbers" << this->_wf->_vertices.size() << std::endl;
	VectorXd dataEngergyMat(outPt_cloud.size());
	dataEngergyMat.setZero();
	CalculateDataEnergy(outPt_cloud, outPt_normals, transMatVec,
		corPt_cloud, dataEngergyMat);
	fio.saveMatrix(dataEngergyMat, "E:\\kinfu\\data\\mat\\dataEngergyMat" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double> jacobiMatData(outPt_cloud.size(), 6 * outPt_cloud.size());
	jacobiMatData.setZero();
	CalculateJacobiMatDataTerm(outPt_cloud, outPt_normals,
		transMatVec, corPt_cloud, jacobiMatData);
	fio.saveMatrix(jacobiMatData, "E:\\kinfu\\data\\mat\\jacobiMatData" + std::to_string(iterationCount) + ".txt");

	SparseMatrix<double>  hessianMatData(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	hessianMatData = (jacobiMatData).transpose()*(jacobiMatData);

	VectorXd regEngergyMat(outPt_cloud.size());
	regEngergyMat.setZero();
	CalRegularizationPenalty(outPt_cloud, transMatVec, weightVec, regEngergyMat);
	fio.saveMatrix(regEngergyMat, "E:\\kinfu\\data\\mat\\regEngergyMat" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double>  jacobiRegMat(outPt_cloud.size(), 6 * outPt_cloud.size());
	jacobiRegMat.setZero();
	CalJacobiMatReg(outPt_cloud, transMatVec, weightVec, jacobiRegMat);

	SparseMatrix<double>  hessianMatReg(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	hessianMatReg.setZero();
	SparseMatrix<double>  jacobiRegMatTrans = (jacobiRegMat).transpose();
	hessianMatReg = jacobiRegMatTrans * (jacobiRegMat);

	fio.saveMatrix(hessianMatReg, "E:\\kinfu\\data\\mat\\hessianMatReg"+std::to_string(iterationCount)+".txt");

	SparseMatrix<double> jacobiMat(outPt_cloud.size(), 6 * outPt_cloud.size());
	jacobiMat.setZero();
	jacobiMat = jacobiMatData + 200 * (jacobiRegMat);
	fio.saveMatrix(jacobiRegMat, "E:\\kinfu\\data\\mat\\jacobiRegMat" + std::to_string(iterationCount) + ".txt");
	SparseMatrix<double> hessianMat(6 * outPt_cloud.size(), 6 * outPt_cloud.size());
	hessianMat = hessianMatData + 200 * hessianMatReg;
	fio.saveMatrix(hessianMat, "E:\\kinfu\\data\\mat\\hessianMat" + std::to_string(iterationCount) + ".txt");
	VectorXd energyMat(outPt_cloud.size());
	energyMat = (dataEngergyMat)+200 * (regEngergyMat);
	fio.saveMatrix(energyMat, "E:\\kinfu\\data\\mat\\energyMat" + std::to_string(iterationCount) + ".txt");

	//hessianMatSparse;
	//(hessianMat);

	Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> choleskeySparse(hessianMat);

	SparseMatrix<double> jacobiMatTrans = jacobiMat.transpose();
	fio.saveMatrix(jacobiMatTrans, "E:\\kinfu\\data\\mat\\jacobiMatTrans" + std::to_string(iterationCount) + ".txt");
	VectorXd B(6 * outPt_cloud.size());
	B.setZero();
	/*
	for (int i = 0; i < outPt_cloud.size(); i++)
	{
		for (int j = 0; j < 6; j++)
		{
			B(i * 6 + j) = jacobiMatTrans.coeffRef(i * 6 + j, i) * energyMat(i);
		}
	}*/
	B = jacobiMatTrans *(energyMat);
	fio.saveMatrix(B,"E:\\kinfu\\data\\mat\\B" + std::to_string(iterationCount) + ".txt");
	//VectorXd deltX = cholesky.solve(B);
	VectorXd deltX2 = B;// choleskeySparse.solve(B);
	fio.saveMatrix(deltX2, "E:\\kinfu\\data\\mat\\deltX2" + std::to_string(iterationCount) + ".txt");
	bool a_solution_exists = (hessianMat*deltX2).isApprox(B, 1);
	std::cout << "cholesky solution " << a_solution_exists << std::endl;
	//if (a_solution_exists)
	//{
	int size = outPt_cloud.size();
	for (int i = 0; i < size; i++)
	{
		MatrixXd se3value(6, 1);
		for (int j = 0; j < 6; j++)
		{
			//std::cout << "step 0.00005 " << a_solution_exists << std::endl;
			double value =  deltX2(i * 6 + j);
			se3value(j, 0) = value;
		}
		Se3<double> deltse3(se3value);
		
		Matrix4f deltTrans = deltse3.GetTransMat4dFromSe3().cast<float>();
		Eigen::Vector3f transPt; 
		TransformPt(this->_wf->_vertices[i], deltTrans, transPt);
		Eigen::Vector3f transDeltPt = transPt - this->_wf->_vertices[i];
		if (transDeltPt.norm() > 30)
		{
			std::cout << "too big step for this point in warpField" << std::endl;
		}
		/*
		std::cout << "SE3 deltTrans start of matrix" << std::endl;
		std::cout << deltTrans(0, 0) << ' ' << deltTrans(0, 1) << ' ' << deltTrans(0, 2) << ' ' << deltTrans(0, 3)<< std::endl;
		std::cout << deltTrans(1, 0) << ' ' << deltTrans(1, 1) << ' ' << deltTrans(1, 2) << ' ' << deltTrans(1, 3) << std::endl;
		std::cout << deltTrans(2, 0) << ' ' << deltTrans(2, 1) << ' ' << deltTrans(2, 2) << ' ' << deltTrans(2, 3) << std::endl;
		std::cout << deltTrans(3, 0) << ' ' << deltTrans(3, 1) << ' ' << deltTrans(3, 2) << ' ' << deltTrans(3, 3) << std::endl;
		std::cout << "end of deltTrans matrix" << std::endl;*/
		this->_wf->_transMatVec[i] = deltTrans * this->_wf->_transMatVec[i];	
		
	}	
	float3Vec newWfPtcloud;
	//delete wf;
	//*wf = (this->_wf);
	int wfsize = (*wf)->_vertices.size();
	std::cout << "the wf point number to be interpolated " << wfsize << std::endl;
	for (int i = 0; i < wfsize; i++)
	{
		this->_wf->InterpolateSE3((*wf)->_vertices[i], (*wf)->_transMatVec[i], 1);
		if (isnan((*wf)->_transMatVec[i](0, 0)))
		{
			std::cout << "update warpfield while there is a point with a nan transMat interpolated" << std::endl;
		}
		/*
		std::cout << "start of matrix  wf->_transMatVec "<<i<< std::endl;
		std::cout << (*wf)->_transMatVec[i](0, 0) << ' ' << (*wf)->_transMatVec[i](0, 1) << ' ' << (*wf)->_transMatVec[i](0, 2) << ' ' << (*wf)->_transMatVec[i](0, 3) << std::endl;
		std::cout << (*wf)->_transMatVec[i](1, 0) << ' ' << (*wf)->_transMatVec[i](1, 1) << ' ' << (*wf)->_transMatVec[i](1, 2) << ' ' << (*wf)->_transMatVec[i](1, 3) << std::endl;
		std::cout << (*wf)->_transMatVec[i](2, 0) << ' ' << (*wf)->_transMatVec[i](2, 1) << ' ' << (*wf)->_transMatVec[i](2, 2) << ' ' << (*wf)->_transMatVec[i](2, 3) << std::endl;
		std::cout << (*wf)->_transMatVec[i](3, 0) << ' ' << (*wf)->_transMatVec[i](3, 1) << ' ' << (*wf)->_transMatVec[i](3, 2) << ' ' << (*wf)->_transMatVec[i](3, 3) << std::endl;
		std::cout << "end of matrix" << std::endl;*/
		Vector3f newPt;
		TransformPt((*wf)->_vertices[i], (*wf)->_transMatVec[i], newPt);
		newWfPtcloud.push_back(newPt);
	}
	string outNgwfPt = "E:\\kinfu\\data\\mat\\outNgwfPt" + std::to_string(iterationCount) + ".ply";
	fio.SavePintsWithNormals(newWfPtcloud, (*wf)->_normals, outNgwfPt);
	delete this->_wf;
	delete this->_defromationTree;
}


