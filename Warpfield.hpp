#ifndef _WARPFIELD_H
#define _WARPFIELD_H
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Dual_quat.h"
#include <vector>
using namespace Eigen;
typedef std::vector<Eigen::Matrix<float, 3, 1>> float3Vec;
typedef std::vector<Eigen::Matrix<float, 4, 4>> SE3Vec;
class Warpfield
{
	
    public:
        Warpfield();
		Warpfield(const float3Vec &vertices, const float3Vec &normals, float radius);
		Warpfield(Vector3f *vertices, Vector3f *normals, int pt_size, float radius);
        ~Warpfield();
		void BuildKdTree();
		void FilterWarpField();
		void AssignWeight();
		void AssignSE3Identity();
    public:
		void init(const float3Vec &vertices, const float3Vec &normals, float radius);
		void ExtendingWarpField(const float3Vec &vertices, const float3Vec &normals,float range);
        void ICPUpdateTransMat(const Eigen::Matrix<float,4,4>& TransMat);
		void ICPUpdateTransMatReborn(const Eigen::Matrix<float, 4, 4> &TransMat);
        Vector3f TransformVertex(const Vector3f &vertex);
        //void TransformVertex(const Vector3f &inpt,const Vector3f &outpt);
		void TransformVertices(const Vector3f &inPt, const Matrix4f &transMat, Vector3f &outPt);
		void FilterPointsWithRange(Vector3f *vertices, Vector3f *normals, int pt_size, 
			float range, float3Vec &newVertices, float3Vec &newNormals);
		void FilterPointsWithRange(const float3Vec &vertices, const float3Vec &normals,
			int pt_size,float range, float3Vec &newVertices, float3Vec &newNormals);
		void FilterBorderPointsWithRange(const float3Vec &vertices, const float3Vec &normals,
			int pt_size, float range, float3Vec &newVertices, float3Vec &newNormals);
		int GetNearPtindex(const Eigen::Vector3f &pt);

		void FilterNoisePointsWithRange(const float3Vec &vertices, const float3Vec &normals,
			int knn, float range, float3Vec &newVertices, float3Vec &newNormals);
		//void FilterPointsWithRange(Vector3f *vertices, int pt_size, float range);
		void InterpolateSE3(const Vector3f &pt, Eigen::Matrix<float, 4,4> &tansMat, int k);
		void InterpolateSE3(const Vector3f &pt, Eigen::Matrix<float, 4, 4> &transMat, float radius);
		float CalTSDFWeight(const Vector3f &pt, int k);
	private:
		bool IsUnsupportedPt(const Eigen::Vector3f &pt);
		float GetDistanceFromNearestPt(const Eigen::Vector3f &pt);
		void ExtendingWeightVec(const float3Vec &insertVertices);
		void ExtendingTransMatVec(const float3Vec &insertVertices, int k);
		float CalQuaternionInterpolationWeight(const Vector3f &pt, const Vector3f &deformationNode, float warpWeight);

    public:
		float _Range = 25;
		int _size;
		float3Vec _vertices;
		float3Vec _normals;
		//pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud;
        std::vector<float> _weightsVec;
		//Se3
		SE3Vec _transMatVec;
		pcl::KdTreeFLANN<pcl::PointXYZ> _kdtree;
        //double _so6d[6];    
        //Eigen::AngleAxisd _so3w;
        //Vector3f _t;
        //Dual_quat<float> _dq;
        //Eigen::Matrix3f _rotation;     
    private:  
};
#endif