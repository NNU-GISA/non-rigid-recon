#include "Warpfield.hpp"
#include "pcl/common/transforms.h"
#include "Dual_quat.h"

Warpfield::Warpfield()
{}
Warpfield::Warpfield(const float3Vec &vertices, const float3Vec &normals,  float range)
{
	std::vector<Vector3f> newVertices;
	std::vector<Vector3f> newNormals;
	int pt_size = vertices.size();
	this->FilterPointsWithRange(vertices, normals, pt_size, range, newVertices, newNormals);
	int new_size = newVertices.size();
	this->_size = new_size;
	this->_vertices = newVertices;
	this->_normals = newNormals;
	BuildKdTree();
	this->AssignWeight();
	this->AssignSE3Identity();
}
Warpfield::Warpfield(Vector3f *vertices, Vector3f *normals, int pt_size, float range)
{
	std::vector<Vector3f> newVertices;
	std::vector<Vector3f> newNormals;

	this->FilterPointsWithRange(vertices, normals, pt_size, range, newVertices, newNormals);
	int new_size = newVertices.size();
	this->_size = new_size;
	this->_vertices = newVertices;
	this->_normals = newNormals;
	BuildKdTree();
	this->AssignWeight();
	this->AssignSE3Identity();
}

Warpfield::~Warpfield()
{}
void Warpfield::init(const float3Vec &vertices, const float3Vec &normals, float radius)
{
	std::vector<Vector3f> newVertices;
	std::vector<Vector3f> newNormals;
	int pt_size = vertices.size();
	this->FilterPointsWithRange(vertices, normals, pt_size, radius, newVertices, newNormals);
	int new_size = newVertices.size();
	this->_size = new_size;
	this->_vertices = newVertices;
	this->_normals = newNormals;
	BuildKdTree();
	this->AssignWeight();
	this->AssignSE3Identity();
}
void Warpfield::BuildKdTree()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < this->_size; ++i)
	{
		pcl::PointXYZ pt(this->_vertices[i](0), this->_vertices[i](1), this->_vertices[i](2));
		pt_cloud->push_back(pt);
	}
	this->_kdtree.setInputCloud(pt_cloud);
	//
	
}
void Warpfield::FilterWarpField()
{
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	float3Vec newPtClouds;
	float3Vec newPtNormals;
	std::vector<float> weightVec;
	std::vector<Eigen::Matrix4f> transMatVec;
	for (int i = 0; i < this->_vertices.size(); ++i)
	{
		pcl::PointXYZ pt(this->_vertices[i](0), this->_vertices[i](1), this->_vertices[i](2));
		this->_kdtree.radiusSearch(pt, 35, pointIdxNKNSearch, pointNKNSquaredDistance);
		int nearNum = pointIdxNKNSearch.size();
		if (nearNum >= 4)
		{
			newPtClouds.push_back(this->_vertices[i]);
			weightVec.push_back(this->_weightsVec[i]);
			newPtNormals.push_back(this->_normals[i]);
			transMatVec.push_back(this->_transMatVec[i]);
		}
	}
	this->_vertices = newPtClouds;
	this->_weightsVec = weightVec;
	int new_size = newPtClouds.size();
	this->_size = new_size;
	this->_normals = newPtNormals;
	this->_transMatVec = transMatVec;
	BuildKdTree();
}
void Warpfield::AssignWeight()
{
	std::vector<int> pointIdxNKNSearch(2);
	std::vector<float> pointNKNSquaredDistance(2);
	for (int i = 0; i < this->_size; ++i)
	{
		pointIdxNKNSearch.clear();
		pointNKNSquaredDistance.clear();
		pcl::PointXYZ pt(this->_vertices[i](0), this->_vertices[i](1), this->_vertices[i](2));
		this->_kdtree.nearestKSearch(pt, 2, pointIdxNKNSearch, pointNKNSquaredDistance);
		this->_weightsVec.push_back(sqrtf(pointNKNSquaredDistance[1]));
	}
}
void Warpfield::AssignSE3Identity()
{
	for (int i = 0; i < this->_size; ++i)
	{
		Eigen::Matrix4f transMat;
		transMat.setIdentity();
		this->_transMatVec.push_back(transMat);
	}
}
void Warpfield::FilterPointsWithRange(Vector3f *vertices, Vector3f *normals, 
	int pt_size, float range, float3Vec &newVertices, float3Vec &newNormals)
{
	Eigen::MatrixXi isValid(pt_size, 1);
	isValid.setConstant(1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < pt_size; ++i)
	{
		pcl::PointXYZ pt(vertices[i](0), vertices[i](1), vertices[i](2));
		pt_cloud->push_back(pt);
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pt_cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	for (int i = 0; i < pt_size; ++i)
	{
		if (isValid(i, 0))
		{
			pointIdxNKNSearch.clear();
			pointNKNSquaredDistance.clear();
			kdtree.radiusSearchT(pt_cloud->points[i],
				double(range), pointIdxNKNSearch, pointNKNSquaredDistance);
			int size = pointIdxNKNSearch.size();
			for(int j = 0; j < size; j++)
			{
				isValid(pointIdxNKNSearch[j], 0) = 0;
			}
		}
		if ((!isnan(vertices[i](0))) && (!isnan(normals[i](0))))
		{
			newVertices.push_back(vertices[i]);
			newNormals.push_back(normals[i]);
		}
	}
}

void Warpfield::FilterPointsWithRange(const float3Vec &vertices, const float3Vec &normals,
	int pt_size, float range, float3Vec &newVertices, float3Vec &newNormals)
{
	if (pt_size == 0)
	{
		return;
	}
	Eigen::MatrixXi isValid(pt_size, 1);
	isValid.setOnes();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < pt_size; ++i)
	{
		pcl::PointXYZ pt(vertices[i](0), vertices[i](1), vertices[i](2));
		pt_cloud->push_back(pt);
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pt_cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	for (int i = 0; i < pt_size; ++i)
	{
		if (isValid(i, 0))
		{
			pointIdxNKNSearch.clear();
			pointNKNSquaredDistance.clear();
			kdtree.radiusSearchT(pt_cloud->points[i],
				double(range), pointIdxNKNSearch, pointNKNSquaredDistance);
			int size = pointIdxNKNSearch.size();
			for (int j = 0; j < size; j++)
			{
				isValid(pointIdxNKNSearch[j], 0) = 0;
			}
			if ((!isnan(vertices[i](0)))&& (!isnan(normals[i](0))))
			{
				newVertices.push_back(vertices[i]);
				newNormals.push_back(normals[i]);
			}	
		}
	}
}
void Warpfield::FilterNoisePointsWithRange(const float3Vec &vertices, const float3Vec &normals,
	int knn, float range, float3Vec &newVertices, float3Vec &newNormals)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < vertices.size(); ++i)
	{
		pcl::PointXYZ pt(vertices[i](0), vertices[i](1), vertices[i](2));
		pt_cloud->push_back(pt);
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pt_cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	for (int i = 0; i < vertices.size(); ++i)
	{
		pointIdxNKNSearch.clear();
		pointNKNSquaredDistance.clear();
		kdtree.radiusSearchT(pt_cloud->points[i],
			double(range), pointIdxNKNSearch, pointNKNSquaredDistance);
		int size = pointIdxNKNSearch.size();
		if (size < knn)
		{
			continue;
		}
		if ((!isnan(vertices[i](0))) && (!isnan(normals[i](0))))
		{
			newVertices.push_back(vertices[i]);
			newNormals.push_back(normals[i]);
		}
		
	}
}
void Warpfield::FilterBorderPointsWithRange(const float3Vec &vertices, const float3Vec &normals,
	int pt_size, float range, float3Vec &newVertices, float3Vec &newNormals)
{
	if (pt_size == 0)
	{
		return;
	}
	Eigen::MatrixXi isValid(pt_size, 1);
	isValid.setOnes();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < pt_size; ++i)
	{
		pcl::PointXYZ pt(vertices[i](0), vertices[i](1), vertices[i](2));
		pt_cloud->push_back(pt);
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(pt_cloud);
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	for (int i = 0; i < pt_size; ++i)
	{
		pointIdxNKNSearch.clear();
		pointNKNSquaredDistance.clear();
		kdtree.radiusSearchT(pt_cloud->points[i],
			double(range), pointIdxNKNSearch, pointNKNSquaredDistance);
		int size = pointIdxNKNSearch.size();
		if (size >= 5)
		{
			if ((!isnan(vertices[i](0))) && (!isnan(normals[i](0))))
			{
				newVertices.push_back(vertices[i]);
				newNormals.push_back(normals[i]);
			}
		}
		
	}
}


int Warpfield::GetNearPtindex(const Eigen::Vector3f &pt)
{
	pcl::PointXYZ newPt(pt(0), pt(1), pt(2));
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	this->_kdtree.nearestKSearch(newPt, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	return pointIdxNKNSearch[0];
}

void Warpfield::ICPUpdateTransMat(const Eigen::Matrix<float, 4, 4> &TransMat)
{
	for (int i = 0; i < this->_size; ++i)
	{
		this->_transMatVec[i] = TransMat*this->_transMatVec[i];
	}
}
void Warpfield::ICPUpdateTransMatReborn(const Eigen::Matrix<float, 4, 4> &TransMat)
{
	Eigen::Matrix4f transMatI;
	transMatI.setIdentity();
	for (int i = 0; i < this->_size; ++i)
	{
		this->_transMatVec[i] = TransMat * transMatI;
			//TransMat*this->_transMatVec[i];
	}
}
void Warpfield::ExtendingWarpField(const float3Vec &vertices, const float3Vec &normals, float range)
{
	float3Vec newVertices;
	float3Vec newNormals;
	for (int i = 0; i < vertices.size(); ++i)
	{
		if (this->IsUnsupportedPt(vertices[i]))
		{
			newVertices.push_back(vertices[i]);
			newNormals.push_back(normals[i]);
		}
	}
	if (vertices.size() == 0)
	{
		return;
	}
	float3Vec insertVertices0;
	float3Vec insertNormals0;
	////bug!!! solved
	this->FilterPointsWithRange(newVertices, newNormals, newVertices.size(), range,
		insertVertices0, insertNormals0);

	float3Vec insertVertices;
	float3Vec insertNormals;
	this->FilterBorderPointsWithRange(insertVertices0, insertNormals0,
	insertVertices0.size(), 45, insertVertices, insertNormals);

	this->ExtendingTransMatVec(insertVertices, 5);
	int insertPt_size = insertVertices.size();
	std::cout << "Extending point size " << insertPt_size << std::endl;
	for (int i = 0; i < insertPt_size; ++i)
	{
		this->_vertices.push_back(insertVertices[i]);
		this->_normals.push_back(insertNormals[i]);
	}
	this->_size = this->_size + insertPt_size;
	this->BuildKdTree();
	this->ExtendingWeightVec(insertVertices);
}
bool Warpfield::IsUnsupportedPt(const Eigen::Vector3f &pt)
{
	float distance = this->GetDistanceFromNearestPt(pt);
	if (distance <= this->_Range)
	{
		return false;
	}
	else
	{
		return true;
	}
}
float  Warpfield::GetDistanceFromNearestPt(const Eigen::Vector3f &pt)
{
	pcl::PointXYZ newpt(pt(0), pt(1), pt(2));
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	this->_kdtree.nearestKSearch(newpt, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	return sqrtf(pointNKNSquaredDistance[0]);
}
void Warpfield::ExtendingWeightVec(const float3Vec &insertVertices)
{
	int size = insertVertices.size();
	std::vector<int> pointIdxNKNSearch(2);
	std::vector<float> pointNKNSquaredDistance(2);
	for (int i = 0; i < size; ++i)
	{
		pointIdxNKNSearch.clear();
		pointNKNSquaredDistance.clear();
		pcl::PointXYZ newpt(insertVertices[i](0), insertVertices[i](1), insertVertices[i](2));
		this->_kdtree.nearestKSearch(newpt,2, pointIdxNKNSearch, pointNKNSquaredDistance);
		this->_weightsVec.push_back(sqrtf(pointNKNSquaredDistance[1]));
	}
}
void Warpfield::ExtendingTransMatVec(const float3Vec &insertVertices,int k)
{
	int pt_size = insertVertices.size();
	for (int i = 0; i < pt_size; ++i)
	{
		Eigen::Matrix4f transMat;
		this->InterpolateSE3(insertVertices[i], transMat, k);
		this->_transMatVec.push_back(transMat);
	}
}
void Warpfield::InterpolateSE3(const Vector3f &pt, Eigen::Matrix<float, 4, 4> &transMat, int k)
{
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	pcl::PointXYZ newpt(pt(0), pt(1), pt(2));
	Dual_quat<float> dq(this->_transMatVec[pointIdxNKNSearch[0]]);
	float weight = this->CalQuaternionInterpolationWeight(pt,
		this->_vertices[pointIdxNKNSearch[0]], this->_weightsVec[pointIdxNKNSearch[0]]);
	dq = dq * weight;
	this->_kdtree.nearestKSearch(newpt, k, pointIdxNKNSearch, pointNKNSquaredDistance);
	for (int i = 1; i < k; ++i)
	{
		float weight = this->CalQuaternionInterpolationWeight(pt, 
			this->_vertices[pointIdxNKNSearch[i]], this->_weightsVec[pointIdxNKNSearch[i]]);
		Dual_quat<float> dqi(this->_transMatVec[pointIdxNKNSearch[i]]);
		if (isnan(this->_transMatVec[pointIdxNKNSearch[i]](0, 0)))
		{
			std::cout << "invalid intepolation nearest pt with nan transMat" << std::endl;
		}
		dq = dq + dqi*weight;
	}
	dq.Normalize();
	transMat = dq.GetTransMat4d();
	if (isnan(transMat(0, 0)))
	{
		std::cout << "invalid intepolation result with nan" << std::endl;
	}
	Vector3f pt0;
	Vector3f pt1;
	this->TransformVertices(pt, transMat, pt0);
	this->TransformVertices(pt, this->_transMatVec[0], pt1);
	Vector3f pt2 = pt1- pt0;
	float num = pt2.norm();
	if ( isnan(transMat(0, 0)))
	{
		//Dual_quat<float> newdq;
		std::cout << "there is a strange point Interpolated" << std::endl;
		for (int i = 0; i < k; ++i)
		{
			std::cout << i << " nearest point " << pointIdxNKNSearch[i]<< std::endl;
			float weight = this->CalQuaternionInterpolationWeight(pt,
				this->_vertices[pointIdxNKNSearch[i]], this->_weightsVec[pointIdxNKNSearch[i]]);
			std::cout << i << " nearest point wf weight " << this->_weightsVec[pointIdxNKNSearch[i]] << std::endl;
			std::cout << i << " nearest point dq weight " << weight << std::endl;
			Dual_quat<float> dqi(this->_transMatVec[pointIdxNKNSearch[i]]);
			Eigen::Matrix4f newtranMat = this->_transMatVec[pointIdxNKNSearch[i]];
			std::cout << newtranMat(0, 0) << " " << newtranMat(0, 1) << " " << newtranMat(0, 2) << " " << newtranMat(0, 3) << " "
				<< newtranMat(1, 0) << " " << newtranMat(1, 1) << " " << newtranMat(1, 2) << " " << newtranMat(1, 3) << " "
				<< newtranMat(2, 0) << " " << newtranMat(2, 1) << " " << newtranMat(2, 2) << " " << newtranMat(2, 3) << " "
				<< newtranMat(3, 0) << " " << newtranMat(3, 1) << " " << newtranMat(3, 2) << " " << newtranMat(3, 3) << std::endl;
			//newdq = newdq + dqi*weight;
		}

		std::cout << transMat(0, 0) << " " << transMat(0, 1) << " " << transMat(0, 2) << " " << transMat(0, 3) << " "
			<< transMat(1, 0) << " " << transMat(1, 1) << " " << transMat(1, 2) << " " << transMat(1, 3) << " "
			<< transMat(2, 0) << " " << transMat(2, 1) << " " << transMat(2, 2) << " " << transMat(2, 3) << " "
			<< transMat(3, 0) << " " << transMat(3, 1) << " " << transMat(3, 2) << " " << transMat(3, 3) << std::endl;

	}

}
void Warpfield::InterpolateSE3(const Vector3f &pt, Eigen::Matrix<float, 4, 4> &transMat, float radius )
{
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	pcl::PointXYZ newpt(pt(0), pt(1), pt(2));
	this->_kdtree.radiusSearch(newpt, radius, pointIdxNKNSearch, pointNKNSquaredDistance);
	if (pointIdxNKNSearch.size() == 0)
	{
		this->_kdtree.nearestKSearch(newpt, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	}
	Dual_quat<float> dq(this->_transMatVec[pointIdxNKNSearch[0]]);
	float weight = this->CalQuaternionInterpolationWeight(pt,
		this->_vertices[pointIdxNKNSearch[0]], this->_weightsVec[pointIdxNKNSearch[0]]);
	dq = dq * weight;
	
	for (int i = 1; i < pointIdxNKNSearch.size(); ++i)
	{
		float weight = this->CalQuaternionInterpolationWeight(pt,
			this->_vertices[pointIdxNKNSearch[i]], this->_weightsVec[pointIdxNKNSearch[i]]);
		Dual_quat<float> dqi(this->_transMatVec[pointIdxNKNSearch[i]]);
		if (isnan(this->_transMatVec[pointIdxNKNSearch[i]](0, 0)))
		{
			std::cout << "invalid intepolation nearest pt with nan transMat" << std::endl;
		}
		dq = dq + dqi*weight;
	}
	dq.Normalize();
	transMat = dq.GetTransMat4d();
	if (isnan(transMat(0, 0)))
	{
		std::cout << "invalid intepolation result with nan" << std::endl;
	}
}
float Warpfield::CalTSDFWeight(const Vector3f &pt, int k)
{
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	pcl::PointXYZ newpt(pt(0), pt(1), pt(2));
	this->_kdtree.nearestKSearch(newpt, k, pointIdxNKNSearch, pointNKNSquaredDistance);
	float sum = 0.0;
	for (int i = 0; i < k; ++i)
	{
		sum += sqrt(pointNKNSquaredDistance[i]);
	}
	return sum / float(k);
}
float Warpfield::CalQuaternionInterpolationWeight(const Vector3f &pt, const Vector3f &deformationNode, float warpWeight)
{
	Vector3f deltPt = deformationNode - pt;
	//std::cout << "deltPt xyz" << deltPt(0) << " " << deltPt(1) << " " << deltPt(2) << std::endl;
	float num = -(deltPt.squaredNorm()) / float(2 * warpWeight * warpWeight);
	float weight = 0.001+ exp(num);
	if (weight == 0)
	{
		std::cout << "deltPt xyz" << deltPt(0) << " " << deltPt(1) << " " << deltPt(2) << std::endl;
		std::cout << "warpWeight " << warpWeight << std::endl;
		std::cout << "deltPt.squaredNorm() " << deltPt.squaredNorm() << std::endl;
		std::cout << "deltPt.squaredNorm() " << num << std::endl;
	}
	return weight;
}
Vector3f Warpfield::TransformVertex(const Vector3f &vertex)
{
    Vector3f pt;
    return pt;
}
    
  
void Warpfield::TransformVertices(const Vector3f &inPt, const Matrix4f &transMat, Vector3f &outPt)
{
	Matrix3f rotationMat = transMat.block(0, 0, 3, 3);
	Vector3f translationMat = transMat.block(0, 3, 3, 1);
	outPt = rotationMat* inPt + translationMat;
}
/*
void Warpfield::TranformVertex(const pcl::PointXYZLNormal &srcPt, pcl::PointXYZLNormal &tarPt)
{
    pcl::transformPointWithNormal(srcPt, tarPt, this->GetTransMat());
}

void Warpfield::TranformPt_cloud(const pcl::PointCloud<pcl::PointXYZLNormal>Ptr ptSrc_cloud,
pcl::PointCloud<pcl::PointXYZLNormal>Ptr ptTar_cloud)
{
    pcl::transformPointCloudWithNormals(ptSrc_cloud,ptTar_cloud,this->GetTransMat());
}
*/
