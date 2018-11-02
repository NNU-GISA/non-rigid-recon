#ifndef TSDF_H
#define TSDF_H
#include "Eigen\Dense"
#include "Vectors.h"
#include "CIsoSurface.h"
#include <vector>
#include <string>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include "Warpfield.hpp"
#include "NewtonGuassSolver.hpp"
#define Max(a,b) (a) > (b)? (a) : (b)
#define Min(a,b) (a) < (b) ? (a) : (b)
class TSDF
{
public:
	
	const float _MaxValue = 30.0;
	const float _MinValue = -30.0;
	//const Eigen::Matrix3f cameraIntr1 = {525,0,319.5,0,525,239.5,0,0,1};


	TSDF();

	TSDF(int resolution_x, int resolution_y, int resolution_z, float cellLength);
	TSDF(int resolution_x, int resolution_y, int resolution_z, float cellLength, float offsetz);

	~TSDF();

	void SetCameraIntr(const Eigen::Matrix3f &cameraIntr);
	
	void CalculateTruncatedSignedValue(int depth_img_width, int depth_img_height, float *depth_vec);
	void CalTSDFLive(int depth_img_width, int depth_img_height, float *depth_vec);
	//3d reconstruction with marching cubes, get surface and point clouds with normals 
	void ReconWithMarchingCubes();
	bool ExtractPointsWithNormalsFromMesh( std::vector<Eigen::Vector3f>&pt_clouds,std::vector<Eigen::Vector3f>&pt_normals);
	void UpdateWarpFieldWithNgSolver(int img_width, int img_height, float *depth, 
		int iterationCount, const float3Vec &mcPts, const float3Vec &mcNormals);
	void AssignTSDFWeight();
	bool RetrivePointByIndex(int x, int y, int z, Eigen::Vector3f &pt);
	bool RetrivePointByPosition(float x, float y, float z, Eigen::Vector3i &pt);
	float RetriveTSDFValue(int x, int y, int z);
	
	const void GetPointClouds(POINT3DXYZ* pt_cloud);
	const void SaveMesh(const std::string &path);
	const void SaveMeshWithNormal(const std::string &path);
	const void SavePintsWithNormals(const std::vector<Eigen::Vector3f>&pt_clouds,
		const std::vector<Eigen::Vector3f>&pt_normals, const std::string &path);
	//fusion live frame
	void ExtractLivePt(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud);
	void TSDFUpdate();
	void TSDFFusion();
	void UpdateTSDF(const TSDF& tsdf2);
	void TransformVertices(const Vector3f &inPt, const Matrix4f &transMat, Vector3f &outPt);
private:
	void SetVexelSize(int resolution_x, int resolution_y, int resolution_z, float cellLength);

public:
	float* _valueVec;
	float* _weightVec;  
	float _x_bound_down;
	float _x_bound_up;
	float _y_bound_down;
	float _y_bound_up;
	float _z_bound_down;
	float _z_bound_up;
	CIsoSurface<float> _marchingCubesTool;
	Warpfield* _warpfield;
	NewtonGuassSolver* _ngSolver;
	float* _valueVecLive;
	float* _weightVecLive;
	int _knn;
private:
	int _resolution_x;
	int _resolution_y;
	int _resolution_z;
	float _cellLength;
	bool _IsMCTInitialzed;
	Eigen::Matrix3f _cameraIntrMat;
	std::vector<Eigen::Vector3f> _pt_cloud;
	std::vector<Eigen::Vector3f> _normal_cloud;
	POINT3DXYZ _TSDFStartPtOffset;// = { -640, -640, 500 };
	bool _hasCameraIntr;
	float _wfRadius = 25;
	float _MaxWeight = 250;
};

#endif