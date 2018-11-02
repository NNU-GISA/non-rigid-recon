#include "TSDF.h"
#include <fstream>
TSDF::TSDF()
{
	this->SetVexelSize(256, 256, 256, 5);
	this->_valueVec = new float[256 * 256 * 256];
	this->_weightVec = new float[256 * 256 * 256];
	//this->_cameraIntrMat.setIdentity();
	this->_hasCameraIntr = false;
}

TSDF::TSDF(int resolution_x, int resolution_y, int resolution_z, float cellLength)
{
	SetVexelSize(resolution_x, resolution_y, resolution_z, cellLength);
	float offsetx = resolution_x / 2 * cellLength;
	float offsety = resolution_y / 2 * cellLength;
	float offsetz = 500.0;
	this->_TSDFStartPtOffset = { -offsetx, -offsety, offsetz };
	this->_x_bound_down = -offsetx;
	this->_x_bound_up = offsetx;
	this->_y_bound_down = -offsetx;
	this->_y_bound_up = offsety;
	this->_z_bound_down = 500;
	this->_z_bound_up = 500 + resolution_z *cellLength;
	this->_valueVec = new float[resolution_x * resolution_y * resolution_z];
	this->_weightVec = new float[resolution_x * resolution_y * resolution_z];

	this->_valueVecLive = new float[resolution_x * resolution_y * resolution_z];
	this->_weightVecLive = new float[resolution_x * resolution_y * resolution_z];
	this->_hasCameraIntr = false;
}
TSDF::TSDF(int resolution_x, int resolution_y, int resolution_z, float cellLength, float offsetz)
{
	SetVexelSize(resolution_x, resolution_y, resolution_z, cellLength);
	float offsetx = resolution_x / 2 * cellLength;
	float offsety = resolution_y / 2 * cellLength;
	//float offsetz = 500.0;
	this->_TSDFStartPtOffset = { -offsetx, -offsety, offsetz };
	this->_x_bound_down = -offsetx;
	this->_x_bound_up = offsetx;
	this->_y_bound_down = -offsetx;
	this->_y_bound_up = offsety;
	this->_z_bound_down = offsetz;
	this->_z_bound_up = offsetz + resolution_z *cellLength;
	this->_valueVec = new float[resolution_x * resolution_y * resolution_z];
	this->_weightVec = new float[resolution_x * resolution_y * resolution_z];
	this->_valueVecLive = new float[resolution_x * resolution_y * resolution_z];
	this->_weightVecLive = new float[resolution_x * resolution_y * resolution_z];
	this->_hasCameraIntr = false;
}

TSDF::~TSDF()
{
}

void TSDF::SetCameraIntr(const Eigen::Matrix3f &cameraIntr)
{
	this->_cameraIntrMat = cameraIntr;
	this->_hasCameraIntr = true;
}

void TSDF::SetVexelSize(int resolution_x, int resolution_y, int resolution_z, float cellLength)
{
	this->_resolution_x = resolution_x;
	this->_resolution_y = resolution_y;
	this->_resolution_z = resolution_z;
	this->_cellLength = cellLength;
}


void TSDF::CalculateTruncatedSignedValue(int depth_img_width, int depth_img_height, float *depth_vec)
{
	float depth_limit = this->_TSDFStartPtOffset.z + this->_resolution_z* this->_cellLength;
	int slice = this->_resolution_x * this->_resolution_y;
	for (int x = 0; x < this->_resolution_x; ++x)
	{
		for (int y = 0; y < this->_resolution_y; ++y)
		{
			for (int z = 0; z < this->_resolution_z; ++z)
			{
				int tsdfValueIndex = z * slice
					+ y * this->_resolution_x + x;
				Eigen::Vector3f point;
				if (this->RetrivePointByIndex(x, y, z, point))
				{
					Eigen::Vector3f img_pt = this->_cameraIntrMat * point;
					if (img_pt(2) != 0)
					{
						int u = int(img_pt(0) / img_pt(2));
						int v = int(img_pt(1) / img_pt(2));
						if (u >= 0 && u < depth_img_width && v >= 0 && v < depth_img_height)
						{
							int imageIndex = v*depth_img_width + u;
							if (depth_limit < depth_vec[imageIndex] || depth_vec[imageIndex] == 0.0)
							{
								this->_valueVec[tsdfValueIndex] = -1.0;
							}
							else
							{
								float tsdf_value = point(2) - depth_vec[imageIndex];
								tsdf_value = Min((float(1.0)), (tsdf_value / this->_MaxValue));
								tsdf_value = Max(-1.0, tsdf_value);
								this->_valueVec[tsdfValueIndex] = tsdf_value;
							}
							//	? 0.0: depth_vec[imageIndex];
						}
						else
						{
							this->_valueVec[tsdfValueIndex] = -1.0;
						}
					}
					else
					{
						this->_valueVec[tsdfValueIndex] = -1.0;
					}
				}
			}
		}
	}
}

void TSDF::CalTSDFLive(int depth_img_width, int depth_img_height, float *depth_vec)
{
	float depth_limit = this->_TSDFStartPtOffset.z + this->_resolution_z* this->_cellLength;
	int slice = this->_resolution_x * this->_resolution_y;
	for (int x = 0; x < this->_resolution_x; ++x)
	{
		for (int y = 0; y < this->_resolution_y; ++y)
		{
			for (int z = 0; z < this->_resolution_z; ++z)
			{
				int tsdfValueLiveIndex = z * slice
					+ y * this->_resolution_x + x;
				Eigen::Vector3f point;
				if (this->RetrivePointByIndex(x, y, z, point))
				{
					Eigen::Matrix4f transMat;
					this->_warpfield->InterpolateSE3(point, transMat, this->_knn);
					Eigen::Vector3f newpoint;
					this->TransformVertices(point, transMat, newpoint);
					
					float weight = this->_warpfield->CalTSDFWeight(newpoint, this->_knn);
					this->_weightVecLive[tsdfValueLiveIndex] = weight;

					Eigen::Vector3f img_pt = this->_cameraIntrMat * newpoint;
					if (img_pt(2) != 0)
					{
						int u = int(img_pt(0) / img_pt(2));
						int v = int(img_pt(1) / img_pt(2));
						if (u >= 0 && u < depth_img_width && v >= 0 && v < depth_img_height)
						{
							int imageIndex = v*depth_img_width + u;
							if (depth_limit < depth_vec[imageIndex] || depth_vec[imageIndex] == 0.0)
							{
								this->_valueVecLive[tsdfValueLiveIndex] = -1.0;
							}
							else
							{
								float tsdf_value = point(2) - depth_vec[imageIndex];
								tsdf_value = Min((float(1.0)), (tsdf_value / this->_MaxValue));
								tsdf_value = Max(-1.0, tsdf_value);
								this->_valueVecLive[tsdfValueLiveIndex] = tsdf_value;
							}
						}
						else
						{
							this->_valueVecLive[tsdfValueLiveIndex] = -1.0;
						}
					}
					else
					{
						this->_valueVecLive[tsdfValueLiveIndex] = -1.0;
					}
				}
			}
		}
	}
}

void TSDF::UpdateWarpFieldWithNgSolver(int img_width, int img_height, float *depth, int iterationCount,
	const float3Vec &mcPts, const float3Vec &mcNormals)
{
	this->_ngSolver = new NewtonGuassSolver(this->_cameraIntrMat, img_width, img_height, depth);
	std::cout << "NG solver to optimize " << iterationCount << " frame" << std::endl;
	this->_ngSolver->NewtonGaussOptimizer2(&(this->_warpfield), iterationCount, mcPts, mcNormals);
	//this->_ngSolver->NewtonGaussOptimizer2(&(this->_warpfield), iterationCount + 100, mcPts, mcNormals);
	//this->_ngSolver->NewtonGaussOptimizer2(&(this->_warpfield), iterationCount + 10000, mcPts, mcNormals);
	//this->_ngSolver->NewtonGaussOptimizer(&(this->_warpfield), iterationCount + 100);
	//this->_ngSolver->NewtonGaussOptimizer(&(this->_warpfield), iterationCount + 1000);
	//this->_ngSolver->NewtonGaussOptimizer(this->_warpfield);
	delete this->_ngSolver;
}

bool TSDF::RetrivePointByIndex(int x, int y, int z, Eigen::Vector3f &pt)
{
	//Eigen::Vector3f point;
	if (x >= 0 && x < this->_resolution_x && y >= 0 && y < this->_resolution_y && z >= 0 && z < this->_resolution_z)
	{
		pt(0) = this->_TSDFStartPtOffset.x + x* this->_cellLength;
		pt(1) = this->_TSDFStartPtOffset.y + y * this->_cellLength;
		pt(2) = this->_TSDFStartPtOffset.z + z * this->_cellLength;
		return true;
	}
	else
	{
		return false;
	}
}
bool TSDF::RetrivePointByPosition(float x, float y, float z, Eigen::Vector3i &pt)
{
	//Eigen::Vector3i point;
	pt(0) = (int)((x - this->_TSDFStartPtOffset.x) / this->_cellLength + 0.5);
	pt(1) = (int)((y - this->_TSDFStartPtOffset.y) / this->_cellLength + 0.5);
	pt(2) = (int)((z - this->_TSDFStartPtOffset.z) / this->_cellLength + 0.5);
	if (pt(0) >= 0 && pt(0) < this->_resolution_x && pt(1) >= 0 && pt(1) < this->_resolution_y && pt(2) >= 0 && pt(2) < this->_resolution_z)
	{
		//pt(0) = this->_TSDFStartPtOffset.x + x* this->_cellLength;
		//pt(1) = this->_TSDFStartPtOffset.y + y * this->_cellLength;
		//pt(2) = this->_TSDFStartPtOffset.z + z * this->_cellLength;
		return true;
	}
	else
	{
		return false;
	}
}
//3d reconstruction with marching cubes, get surface and point clouds with normals 
void TSDF::ReconWithMarchingCubes()
{
	if (this->_marchingCubesTool.IsSurfaceValid())
	{
		this->_marchingCubesTool.DeleteSurface();
	}
	this->_marchingCubesTool.GenerateSurface(this->_valueVec, 0.0,
		this->_resolution_x - 1, this->_resolution_y - 1, this->_resolution_z - 1,
		this->_cellLength, this->_cellLength, this->_cellLength);
	this->_IsMCTInitialzed = true;
	/*
	if (this->_marchingCubesTool.IsSurfaceValid())
	{
		this->_marchingCubesTool;
	}*/
}
/*
void TSDF::InitWarpField()
{
	if (!this->_IsMCTInitialzed)
	{
		std::cout << "WarpField initialized failed!" << std::endl;
		return;
	}
	std::vector<Eigen::Vector3f> pt_cloud;
	std::vector<Eigen::Vector3f> normal_cloud;
	this->ExtractPointsWithNormalsFromMesh(pt_cloud, normal_cloud);

	Warpfield newwf(pt_cloud,normal_cloud,this->_wfRadius);
	//this->_warpfield();
}*/

void TSDF::TransformVertices(const Vector3f &inPt, const Matrix4f &transMat, Vector3f &outPt)
{
	Matrix3f rotationMat = transMat.block(0, 0, 3, 3);
	Vector3f translationMat = transMat.block(0, 3, 3, 1);
	outPt = rotationMat* inPt + translationMat;
}
void TSDF::AssignTSDFWeight()
{
	
	int slice = this->_resolution_x* this->_resolution_y;
	for (int z = 0; z < this->_resolution_z; ++z)
	{
		for (int y = 0; y < this->_resolution_y; ++y)
		{
			for (int x = 0; x < this->_resolution_x; ++x)
			{
				int index = z* slice + y * this->_resolution_x + x; 
				Eigen::Vector3f pt;
				this->RetrivePointByIndex(x, y, z, pt);
				float weight = this->_warpfield->CalTSDFWeight(pt,this->_knn);
				this->_weightVec[index] = weight;
			}
		}
	}
}
void TSDFUpdate()
{

}
void TSDF::TSDFFusion()
{
	int slice = this->_resolution_x* this->_resolution_y;
	for (int z = 0; z < this->_resolution_z; ++z)
	{
		for (int y = 0; y < this->_resolution_y; ++y)
		{
			for (int x = 0; x < this->_resolution_x; ++x)
			{
				int index = z* slice + y * this->_resolution_x + x;
				if (this->_valueVecLive[index] > -1 && this->_valueVecLive[index] < 1)
				{
					float value = (this->_valueVec[index] * this->_weightVec[index] +
						this->_valueVecLive[index] * this->_weightVecLive[index])
						/ (this->_weightVec[index] + this->_weightVecLive[index]);
					this->_valueVec[index] = value;
					float weight = Min(this->_weightVec[index] + this->_weightVecLive[index], this->_MaxWeight);
					this->_weightVec[index] = weight;
				}
			}
		}
	}
}
void TSDF::UpdateTSDF(const TSDF& tsdf2)
{
	int slice = this->_resolution_x* this->_resolution_y;
	for (int z = 0; z < this->_resolution_z; ++z)
	{
		for (int y = 0; y < this->_resolution_y; ++y)
		{
			for (int x = 0; x < this->_resolution_x; ++x)
			{
				int index = z* slice + y * this->_resolution_x + x;
				if (tsdf2._valueVec[index] > -1)
				{
					float value = (this->_valueVec[index] * this->_weightVec[index] +
						tsdf2._valueVec[index] * tsdf2._weightVec[index])
						/ (this->_weightVec[index] + tsdf2._weightVec[index]);
					float weight = Min(this->_weightVec[index]+ tsdf2._weightVec[index], this->_MaxWeight);
					this->_weightVec[index] = weight;
				}	
			}
		}
	}
}
bool TSDF::ExtractPointsWithNormalsFromMesh(std::vector<Eigen::Vector3f>&pt_clouds,
	std::vector<Eigen::Vector3f>&pt_normals)
{
	std::vector<Eigen::Vector3f> new_pt_clouds;
	std::vector<Eigen::Vector3f> new_pt_normals;
	Vector3f offset;
	offset(0) = this->_TSDFStartPtOffset.x;
	offset(1) = this->_TSDFStartPtOffset.y;
	offset(2) = this->_TSDFStartPtOffset.z;

	this->_marchingCubesTool.GetMeshPointsWithNormals(new_pt_clouds, new_pt_normals, offset);
	int pt_size = new_pt_clouds.size();
	int normal_size = new_pt_normals.size();
	if (pt_size != normal_size)
	{
		std::cout << "nromal number doesn't match point number" << std::endl;
		return false;
	}
	for (int i = 0; i < pt_size; i++)
	{
		Eigen::Vector3i pt_index;
		this->RetrivePointByPosition(new_pt_clouds[i](0), 
			new_pt_clouds[i](1) , new_pt_clouds[i](2), pt_index);
		float tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
		if (tsdf_value > 0.2 && tsdf_value < -0.2)
		{
			continue;
		}
		pt_index(0) = pt_index(0) + 1;
		if (pt_index(0) < this->_resolution_x)
		{
			tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
			if (tsdf_value > 0.2 && tsdf_value < -0.2)
			{
				continue;
			}
		}
		pt_index(0) = pt_index(0) - 2;
		if (pt_index(0) < this->_resolution_x && pt_index(0) >= 0)
		{
			tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
			if (tsdf_value > 0.2 && tsdf_value < -0.2)
			{
				continue;
			}
		}
		pt_index(0) = pt_index(0) +1;

		pt_index(1) = pt_index(1) + 1;
		if (pt_index(1) < this->_resolution_y)
		{
			tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
			if (tsdf_value > 0.2 && tsdf_value < -0.2)
			{
				continue;
			}
		}
		pt_index(1) = pt_index(1) - 2;
		if (pt_index(1) < this->_resolution_y && pt_index(1) >= 0)
		{
			tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
			if (tsdf_value > 0.2 && tsdf_value < -0.2)
			{
				continue;
			}
		}
		pt_index(1) = pt_index(1) + 1;

		pt_index(2) = pt_index(2) + 1;
		if (pt_index(2) < this->_resolution_z)
		{
			tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
			if (tsdf_value > 0.2 && tsdf_value < -0.2)
			{
				continue;
			}
		}
		pt_index(2) = pt_index(2) - 2;
		if (pt_index(2) < this->_resolution_z && pt_index(2) >= 0)
		{
			tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
			if (tsdf_value > 0.2 && tsdf_value < -0.2)
			{
				continue;
			}
		}
		pt_index(2) = pt_index(2) + 1;
		tsdf_value = this->RetriveTSDFValue(pt_index(0), pt_index(1), pt_index(2));
		if (tsdf_value < 0.2 && tsdf_value > -0.2)
		{
			pt_clouds.push_back(new_pt_clouds[i]);
			pt_normals.push_back(new_pt_normals[i]);
		}
	}
	return true;
}
void TSDF::ExtractLivePt(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud)
{
	int slice = this->_resolution_x * this->_resolution_y;
	for (int x = 0; x < this->_resolution_x; x = x + 2)
	{
		for (int y = 0; y < this->_resolution_y; y = y + 2)
		{
			for (int z = 0; z < this->_resolution_z; z = z + 2)
			{
				int index = z* slice + y * 256 + x;
				if (abs(this->_valueVecLive[index]) <= 1)
				{
					Eigen::Vector3f pt_f;
					if (this->RetrivePointByIndex(x, y, z, pt_f))
					{
						if (this->_valueVecLive[index] == -1)
						{
							continue;
						}
						else
						{
							pcl::PointXYZRGB pt;
							pt.x = pt_f(0);
							pt.y = pt_f(1);
							pt.z = pt_f(2);
							pt.r = 127 * (1 + this->_valueVecLive[index]);
							pt.g = 255 * (1 - abs(this->_valueVecLive[index]));
							pt.b = 127 * (1 - this->_valueVecLive[index]);
							ptcloud->push_back(pt);
						}
					}
				}
			}
		}
	}
}
float TSDF::RetriveTSDFValue(int x, int y, int z)
{
	return this->_valueVec[z*this->_resolution_x*this->_resolution_y + y * this->_resolution_x + x];
}

const void TSDF::GetPointClouds(POINT3DXYZ* pt_cloud)
{

}

const void TSDF::SaveMesh(const std::string &path)
{
	Vector3f offset;
	offset(0) = this->_TSDFStartPtOffset.x;
	offset(1) = this->_TSDFStartPtOffset.y;
	offset(2) = this->_TSDFStartPtOffset.z;
	this->_marchingCubesTool.SaveMeshPly(path, offset);
}
const void TSDF::SaveMeshWithNormal(const std::string &path)
{
	Vector3f offset;
	offset(0) = this->_TSDFStartPtOffset.x;
	offset(1) = this->_TSDFStartPtOffset.y;
	offset(2) = this->_TSDFStartPtOffset.z;
	this->_marchingCubesTool.SaveMeshPlyWithNormal(path, offset);
}

const void TSDF::SavePintsWithNormals(const std::vector<Eigen::Vector3f>&pt_clouds,
	const std::vector<Eigen::Vector3f>&pt_normals, const std::string &path)
{
	std::ofstream filestream;
	filestream.open(path);
	//wrting ply header
	filestream << "ply" << std::endl;
	filestream << "format ascii 1.0" << std::endl;
	filestream << "comment made via Marching cubes" << std::endl;
	int point_number = pt_clouds.size();
	filestream << "element vertex " << point_number << std::endl;
	filestream << "property float x" << std::endl;
	filestream << "property float y" << std::endl;
	filestream << "property float z" << std::endl;
	filestream << "property float nx" << std::endl;
	filestream << "property float ny" << std::endl;
	filestream << "property float nz" << std::endl;

	filestream << "element face 0" <<std::endl;
	filestream << "property list uchar int vertex_indices" << std::endl;
	filestream << "end_header" << std::endl;

	for (int i = 0; i < point_number; i++)
	{
		filestream << pt_clouds[i](0) << " " << pt_clouds[i](1) << " " << pt_clouds[i](2) << " "
			<< pt_normals[i](0) << " " << pt_normals[i](1) << " " << pt_normals[i](2)<<std::endl;
	}

}