#include "TSDF.h"
#include "opencv2\highgui.hpp"
#include "opencv2\opencv.hpp"
#include <iostream>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\io\ply_io.h>
#include "MarchingCubes.h"
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include "DenseICP.hpp"
#include <stdio.h>
#include <io.h>
#include "FileIO.h"
typedef Eigen::Matrix3f CameraIntr;

void getFiles(const std::string &path, std::vector<std::string>& files)
{
	//文件句柄  
	intptr_t  hFile = 0;
	//文件信息  
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之  
			//如果不是,加入列表  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
void ConvertDepthImages2Points(const std::string &imgfile, pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud)
{
	float fx = 525.0; //# focal length x
	float fy = 525.0;  //# focal length y
	float cx = 319.5;  //# optical center x
	float cy = 239.5; // # optical center y
	cv::Mat map = cv::imread(imgfile, CV_LOAD_IMAGE_UNCHANGED);
	int width = map.cols;
	int height = map.rows;
	std::cout << "hello" << std::endl;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZ>);
	TSDF tsdf(256, 256, 256, 5.0);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			ushort d = map.at<ushort>(i, j);
			pcl::PointXYZ pt;
			pt.z = float(d / 5.0);
			pt.x = (j - cx) *pt.z / fx;
			pt.y = (i - cy) *pt.z / fy;
			Eigen::Vector3i pt_int;
			if (tsdf.RetrivePointByPosition(pt.x, pt.y, pt.z, pt_int))
			{
				Eigen::Vector3f pt_f;
				tsdf.RetrivePointByIndex(pt_int(0), pt_int(1), pt_int(2), pt_f);
				pcl::PointXYZ new_pt(pt_f(0), pt_f(1), pt_f(2));
				ptcloud->push_back(new_pt);
			}
		}
	}
	//pcl::io::savePLYFile(outfilePath, *ptcloud);
}
void loadDepthImages(const std::string &imgfile, float *depthVec, float scale = 5.0)
{
	cv::Mat map = cv::imread(imgfile, CV_LOAD_IMAGE_UNCHANGED);
	//cv::imshow("window", map);
	int width = map.cols;
	int height = map.rows;
	//depthVec = new float[width *height];
	//cv::Mat adjMap;
	//cv::convertScaleAbs(map, adjMap, 1/10);
	//cv::CV_16UC1 data = map.data;
	std::cout << "hello" << std::endl;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			ushort d = map.at<ushort>(i, j);
			depthVec[width*i + j] = float(d / scale);
		}
	}
	//uchar* data = (uchar*) map.
	//cv::Mat adjMap;
	//cv::convertScaleAbs(map, adjMap, 1/5);
	//cv::waitKey(0);
}
CameraIntr SetCameraIntrf1()
{
	float fx = 525.0;//  # focal length x
	float fy = 525.0;//  # focal length y
	float cx = 319.5;//  # optical center x
	float cy = 239.5;//  # optical center y
	CameraIntr ci;
	ci.setIdentity();
	ci(0, 0) = fx;
	ci(0, 2) = cx;
	ci(1, 1) = fy;
	ci(1, 2) = cy;
	return ci;
}
CameraIntr SetCameraIntrf2()
{
	float fx = 570.342;//  # focal length x
	float fy = 570.342;//  # focal length y
	float cx = 320;//  # optical center x
	float cy = 240;//  # optical center y
	CameraIntr ci;
	ci.setIdentity();
	ci(0, 0) = fx;
	ci(0, 2) = cx;
	ci(1, 1) = fy;
	ci(1, 2) = cy;
	return ci;
}

void tsdftest()
{
	int image_width = 640;
	int image_height = 480;
	float* depth = new float[image_height*image_width];
	std::string depthMapPath = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png";
	loadDepthImages(depthMapPath, depth);
	//TSDF tsdf(512, 512, 512, 5.0);
	TSDF tsdf(256, 256, 256, 5.0);
	CameraIntr Camera1 = SetCameraIntrf1();
	tsdf.SetCameraIntr(Camera1);
	tsdf.CalculateTruncatedSignedValue(image_width, image_height, depth);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	int slice = 256 * 256;
	for (int x = 0; x < 256; x= x+2)
	{
		for (int y = 0; y < 256; y = y+2)
		{
			for (int z = 0; z < 256; z = z+2)
			{
				int index = z* slice + y * 256 + x;
				if (abs(tsdf._valueVec[index]) <= 1)
				{
					Eigen::Vector3f pt_f;
					if (tsdf.RetrivePointByIndex(x, y, z, pt_f))
					{
						if (tsdf._valueVec[index] == -1)
						{
							continue;
						}
						else
						{
							pcl::PointXYZRGB pt;
							pt.x = pt_f(0);
							pt.y = pt_f(1);
							pt.z = pt_f(2);
							pt.r = 127 * (1 + tsdf._valueVec[index]);
							pt.g = 255 * (1 - abs(tsdf._valueVec[index]));
							pt.b = 127 * (1 - tsdf._valueVec[index]);
							ptcloud->push_back(pt);
						}
					}
				}
			}
		}
	}
	
	std::string outPutPath0 = "E:\\kinfu\\data\\mesh_tsdf02_normal.ply";
	std::string outPutPath1 = "E:\\kinfu\\data\\ptcloud_tsdf02.ply";
	//pcl::io::savePLYFile(outPutPath, *ptcloud);
	pcl::io::savePLYFile(outPutPath1, *ptcloud);
	tsdf.ReconWithMarchingCubes();
	tsdf.SaveMeshWithNormal(outPutPath0);
	
	if (tsdf._marchingCubesTool.IsSurfaceValid())
	{
		std::vector<Eigen::Vector3f> pt_clouds;
		std::vector<Eigen::Vector3f> pt_normals;
		if (tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals))
		{
			std::string outPutPath2 = "E:\\kinfu\\data\\ptcloud_tsdf02_normal.ply";
			tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath2);	
		}	
	}
	
}
void testConvertImages()
{
	std::string depthMapPath = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png";
	std::string outPutPath = "E:\\kinfu\\data\\ptCloud00.ply";
	//ConvertDepthImages2Points(depthMapPath, outPutPath);
}
void savemesh(TRIANGLE* triangles, int numTriangles, std::string filePath);
void savemesh2(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud_unique,
	std::vector<Eigen::Vector3i> trianglePtindex, std::string filePath);
void testMC()
{
	int image_width = 640;
	int image_height = 480;
	float* depth = new float[image_height*image_width];
	std::string depthMapPath = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png";
	loadDepthImages(depthMapPath, depth);
	//TSDF tsdf(512, 512, 512, 5.0);
	TSDF tsdf(256, 256, 256, 5.0);
	CameraIntr Camera1 = SetCameraIntrf1();
	tsdf.SetCameraIntr(Camera1);
	tsdf.CalculateTruncatedSignedValue(image_width, image_height, depth);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::savePLYFile(outPutPath1, *ptcloud);
	mp4Vector* pt4_cloud = new mp4Vector[256 * 256 * 256];
	int slice = 256 * 256;
	for (int x = 0; x < 256; x++)
	{
		for (int y = 0; y < 256; y++)
		{
			for (int z = 0; z < 256; z++)
			{
				int index = z* slice + y * 256 + x;
				Eigen::Vector3f pt_f;
				if (tsdf.RetrivePointByIndex(x, y, z, pt_f))
				{
					pt4_cloud[index].val = tsdf._valueVec[index];
					pt4_cloud[index].x = pt_f(0);
					pt4_cloud[index].y = pt_f(1);
					pt4_cloud[index].z = pt_f(2);
					//pcl::PointXYZ pt(pt_f(0), pt_f(1), pt_f(2));
					//ptcloud->push_back(pt);
				}
			}
		}
	}
	int numTriangles = 0;
	TRIANGLE* triangles = MarchingCubes(255, 255, 255, 5, 5, 5, 0.0, pt4_cloud, numTriangles);
	//tsdf.ReconWithMarchingCubes();
	
	std::vector<mpVector> ptVector;
	for (int i = 0; i < numTriangles; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			pcl::PointXYZ new_pt(triangles[i].p[j].x, triangles[i].p[j].y, triangles[i].p[j].z);
			ptcloud->push_back(new_pt);
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(ptcloud);
	int K = 1;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);
	bool* pt_uniqueSign = new bool[ptcloud->size()];
	for (int i = 0; i < ptcloud->size(); i++)
	{
		pt_uniqueSign[i] = 0;
	}
	int* index_vec = new int[ptcloud->size()];
	for (int i = 0; i < ptcloud->size(); i++)
	{
		pointIdxNKNSearch.clear();
		pointNKNSquaredDistance.clear();
		kdtree.nearestKSearch(ptcloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);
		pt_uniqueSign[pointIdxNKNSearch[0]] = 1;
		index_vec[i] = pointIdxNKNSearch[0];
	}
	int* id_vec = new int[ptcloud->size()];
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud_unique(new pcl::PointCloud<pcl::PointXYZ>);
	int ptcount = 0;
	for (int i = 0; i < ptcloud->size(); i++)
	{
		if (pt_uniqueSign[i] == 1)
		{
			id_vec[i] = ptcount;
			ptcloud_unique->push_back(ptcloud->points[i]);
			ptcount++;
		}
		else
		{
			id_vec[i] = -1;
		}
	}
	std::vector<Eigen::Vector3i> trianglePtindex;
	for (int i = 0; i < numTriangles; ++i)
	{
		Eigen::Vector3i ptId;
		ptId(0) = id_vec[index_vec[i * 3]];
		ptId(1) = id_vec[index_vec[i * 3 + 1]];
		ptId(2) = id_vec[index_vec[i * 3 + 2]];
		trianglePtindex.push_back(ptId);
	}
	
	std::string outPutPath0 = "E:\\kinfu\\data\\mesh_tsdf_MC.ply";
	savemesh2(ptcloud_unique, trianglePtindex, outPutPath0);
	//savemesh(triangles, numTriangles, outPutPath0);
	//tsdf.SaveMesh(outPutPath0);
}
void savemesh2(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud_unique,
	std::vector<Eigen::Vector3i> trianglePtindex, std::string filePath)
{
	std::ofstream filestream;
	filestream.open(filePath);
	//wrting ply header
	filestream << "ply" << std::endl;
	filestream << "format ascii 1.0" << std::endl;
	filestream << "comment made via Marching cubes" << std::endl;
	int point_number = ptcloud_unique->size();
	filestream << "element vertex " << point_number << std::endl;
	filestream << "property float x" << std::endl;
	filestream << "property float y" << std::endl;
	filestream << "property float z" << std::endl;
	int triangle_number = trianglePtindex.size();
	filestream << "element face " << triangle_number << std::endl;
	filestream << "property list uchar int vertex_indices" << std::endl;
	filestream << "end_header" << std::endl;

	for (int i = 0; i < point_number; ++i)
	{
		filestream << ptcloud_unique->points[i].x << " "
			<< ptcloud_unique->points[i].y << " "
			<< ptcloud_unique->points[i].z << std::endl;
	}
	for (int i = 0; i < triangle_number; i++)
	{
		filestream << 3 << " "
			<< trianglePtindex[i](0)<< " "
			<< trianglePtindex[i](1) << " "
			<< trianglePtindex[i](2) << std::endl;
	}
	filestream.close();
}
void savemesh(TRIANGLE* triangles, int numTriangles, std::string filePath)
{
	std::ofstream filestream;
	filestream.open(filePath);
	//wrting ply header
	filestream << "ply" << std::endl;
	filestream << "format ascii 1.0" << std::endl;
	filestream << "comment made via Marching cubes" << std::endl;
	int point_number = numTriangles * 3;
	filestream << "element vertex " << point_number << std::endl;
	filestream << "property float x" << std::endl;
	filestream << "property float y" << std::endl;
	filestream << "property float z" << std::endl;
	int triangle_number = numTriangles;
	filestream << "element face " << triangle_number << std::endl;
	filestream << "property list uchar int vertex_indices" << std::endl;
	filestream << "end_header" << std::endl;

	for (int i = 0; i < numTriangles; ++i)
	{
		for (int j = 0; j < 3; j++)
		{
			filestream << triangles[i].p[j].x << " "
				<< triangles[i].p[j].y << " "
				<< triangles[i].p[j].z << std::endl;
		}
	}
	for (int i = 0; i < numTriangles; i++)
	{
		filestream << 3 << " "
			<< 3*i <<" "
			<< 3*i +1 << " "
			<< 3*i +2  << std::endl;
	}
	filestream.close();
}
void ICPtest()
{
	std::string rgbdPath1 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png ";
	std::string rgbdPath2 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.834105.png ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	DenseICP dicp;
	tsdfBound bound = { -128 * 5 ,128*5, -128*5, 128* 5, 500, 500+256*5};
	CameraIntr c1 = SetCameraIntrf1();
	int image_width = 640;
	int image_height = 480;
	float* depth1 = new float[image_height*image_width];
	loadDepthImages(rgbdPath1, depth1);
	dicp.GetDensePtCloud(ptcloud1, bound, image_width, image_height, depth1, c1);
	float *depth2 = new float[image_height*image_width];
	loadDepthImages(rgbdPath2, depth2);
	dicp.GetDensePtCloud(ptcloud2, bound, image_width, image_height, depth2, c1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f tansMat;
	dicp.EstimateTransMat(ptcloud1, ptcloud2, target_ptcloud1, tansMat);



	std::string outPtcloud1 = "E:\\kinfu\\data\\ptcloud_icp_pt1.ply";
	std::string outPtcloud2 = "E:\\kinfu\\data\\ptcloud_icp_pt2.ply";
	std::string outPtcloud3 = "E:\\kinfu\\data\\ptcloud_icp_pt1_registrated.ply";
	pcl::io::savePLYFile(outPtcloud1,*ptcloud1);
	pcl::io::savePLYFile(outPtcloud2,*ptcloud2);
	pcl::io::savePLYFile(outPtcloud3,*target_ptcloud1);
}
void RigidFusiontest()
{
	std::string rgbdPath1 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png ";
	std::string rgbdPath2 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.866730.png ";
	std::string rgbdPath3 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.934244.png ";
	std::string rgbdPath4 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.970244.png ";
	//std::string rgbdPath5 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846648.003091.png ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	DenseICP dicp;
	tsdfBound bound = { -128 * 5 ,128 * 5, -128 * 5, 128 * 5, 500, 500 + 256 * 5 };
	CameraIntr c1 = SetCameraIntrf1();
	int image_width = 640;
	int image_height = 480;
	float* depth1 = new float[image_height*image_width];
	loadDepthImages(rgbdPath1, depth1);
	dicp.GetDensePtCloud(ptcloud1, bound, image_width, image_height, depth1, c1);

	float *depth2 = new float[image_height*image_width];
	loadDepthImages(rgbdPath2, depth2);
	dicp.GetDensePtCloud(ptcloud2, bound, image_width, image_height, depth2, c1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f tansMat;
	dicp.EstimateTransMat(ptcloud1, ptcloud2, target_ptcloud1, tansMat);

	//TSDF 
	TSDF tsdf(256, 256, 256, 5.0);
	tsdf._knn = 5;
	CameraIntr Camera1 = SetCameraIntrf1();
	tsdf.SetCameraIntr(Camera1);
	tsdf.CalculateTruncatedSignedValue(image_width, image_height, depth1);

	tsdf.ReconWithMarchingCubes();
	if (tsdf._marchingCubesTool.IsSurfaceValid())
	{
		std::vector<Eigen::Vector3f> pt_clouds;
		std::vector<Eigen::Vector3f> pt_normals;
		if (tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals))
		{
			tsdf._warpfield = new Warpfield(pt_clouds, pt_normals, 25.0);
			tsdf.AssignTSDFWeight();
			tsdf._warpfield->ICPUpdateTransMat(tansMat);
			std::string outPutPath2 = "E:\\kinfu\\data\\ptcloud_tsdf_normal.ply";
			tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath2);

			std::string outPutWFPath = "E:\\kinfu\\data\\ptcloud_tsdf_warp.ply";
			tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices,tsdf._warpfield->_normals, outPutWFPath);
		}
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud_live(new pcl::PointCloud<pcl::PointXYZRGB>);
		pt_clouds.clear();
		pt_normals.clear();
		tsdf.CalTSDFLive(image_width, image_height, depth2);
		tsdf.ExtractLivePt(ptcloud_live);
		std::string outLivePath = "E:\\kinfu\\data\\ptcloud_tsdf_live.ply";
		pcl::io::savePLYFile(outLivePath, *ptcloud_live);

		tsdf.TSDFFusion();
		tsdf.ReconWithMarchingCubes();
		tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
		std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd.ply";
		tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath3);
		std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal.ply";
		tsdf.SaveMeshWithNormal(foutpath);
	}
}

void RigidFusiontestMultiInput()
{
	std::string filepath = "E:\\kinfu\\data\\testdata\\newtest1";
	std::vector<std::string> pathSet;
	getFiles(filepath, pathSet);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	DenseICP dicp;
	//tsdfBound bound = { -128 * 5 ,128 * 5, -128 * 5, 128 * 5, 500, 500 + 256 * 5 };
	tsdfBound bound = { -64 * 0.005 ,64 * 0.005, -64 * 0.005, 64 * 0.005, 1.200, 1.200 + 128 * 0.005 };
	CameraIntr c1 = SetCameraIntrf1();
	int image_width = 640;
	int image_height = 480;
	float* depth1 = new float[image_height*image_width];
	loadDepthImages(pathSet[0], depth1);
	dicp.GetDensePtCloud(ptcloud1, bound, image_width, image_height, depth1, c1);
	std::string outPutPathwf00 = "E:\\kinfu\\data\\live00.ply";
	pcl::io::savePLYFile(outPutPathwf00, *ptcloud1);
	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//kdtree.setInputCloud(ptcloud1);
	//std::vector<int> pointIdxNKNSearch;
	//std::vector<float> pointNKNSquaredDistance;
	//TSDF 
	TSDF tsdf(128, 128, 128, 0.0050,1.200);
	//TSDF tsdf(256, 256, 256, 5.0);
	tsdf._knn = 5;
	CameraIntr Camera1 = SetCameraIntrf1();
	tsdf.SetCameraIntr(Camera1);
	tsdf.CalculateTruncatedSignedValue(image_width, image_height, depth1);
	tsdf.ReconWithMarchingCubes();
	
	if (tsdf._marchingCubesTool.IsSurfaceValid())
	{
		std::vector<Eigen::Vector3f> pt_clouds;
		std::vector<Eigen::Vector3f> pt_normals;
		if (tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals))
		{
			
			tsdf._warpfield = new Warpfield(pt_clouds, pt_normals, 0.025);
			//tsdf._warpfield->FilterWarpField();
			std::cout << "warpfield point numbers" << tsdf._warpfield->_vertices.size() << std::endl;
			std::string outPutPathwf = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf0.ply";
			tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutPathwf);

			tsdf.AssignTSDFWeight();
			for (int i = 1; i < pathSet.size(); i++)
			{
				float *depth_live = new float[image_height*image_width];
				loadDepthImages(pathSet[i], depth_live);
				pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud_live(new pcl::PointCloud<pcl::PointXYZ>);
				dicp.GetDensePtCloud(ptcloud_live, bound, image_width, image_height, depth_live, c1);
				std::string outLivePath = "E:\\kinfu\\data\\ptcloud_tsdf_live" + std::to_string(i) + ".ply";
				pcl::io::savePLYFile(outLivePath, *ptcloud_live);

				pcl::PointCloud<pcl::PointXYZ>::Ptr registrated_ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
				Eigen::Matrix4f tansMat;

				dicp.EstimateTransMat(ptcloud1, ptcloud_live, registrated_ptcloud1, tansMat);
				std::string outICPMat = "E:\\kinfu\\data\\ICPmat" + std::to_string(i) + ".txt";
				FileIO fio;
				fio.saveMatrix(tansMat.cast<double>(), outICPMat);

				pt_clouds.clear();
				pt_normals.clear();
				float3Vec pt_clouds_filted;
				float3Vec pt_normals_filted;
				tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
				tsdf._warpfield->ICPUpdateTransMat(tansMat);
				tsdf.UpdateWarpFieldWithNgSolver(image_width, image_height, 
					depth_live, i, pt_clouds, pt_normals);
				tsdf.CalTSDFLive(image_width, image_height, depth_live);
				tsdf.TSDFFusion();
				tsdf.ReconWithMarchingCubes();
				tsdf._warpfield->ExtendingWarpField(pt_clouds, pt_normals, 0.025);
				std::string outPutPathwf = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf" + std::to_string(i) + ".ply";
				tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutPathwf);

				std::string outPutPathwf2 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf_trans" + std::to_string(i) + ".ply";
				std::vector<Eigen::Vector3f> pt_clouds_trans;
				std::vector<Eigen::Vector3f> pt_normals_trans;
				for (int k = 0; k < tsdf._warpfield->_vertices.size(); k++)
				{
					Eigen::Vector3f new_pt;
					Eigen::Vector3f new_normal;
					if (isnan(tsdf._warpfield->_transMatVec[k](0,0)))
					{
						std::cout << "start of matrix tsdf._warpfield->_transMatVec[k] " << k<< std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](0, 0) << ' ' << tsdf._warpfield->_transMatVec[k](0, 1) << ' ' << tsdf._warpfield->_transMatVec[k](0, 2) << ' ' << tsdf._warpfield->_transMatVec[k](0, 3) << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](1, 0) << ' ' << tsdf._warpfield->_transMatVec[k](1, 1) << ' ' << tsdf._warpfield->_transMatVec[k](1, 2) << ' ' << tsdf._warpfield->_transMatVec[k](1, 3) << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](2, 0) << ' ' << tsdf._warpfield->_transMatVec[k](2, 1) << ' ' << tsdf._warpfield->_transMatVec[k](2, 2) << ' ' << tsdf._warpfield->_transMatVec[k](2, 3) << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](3, 0) << ' ' << tsdf._warpfield->_transMatVec[k](3, 1) << ' ' << tsdf._warpfield->_transMatVec[k](3, 2) << ' ' << tsdf._warpfield->_transMatVec[k](3, 3) << std::endl;
						std::cout << "end of matrix" << std::endl;
					}
					tsdf.TransformVertices(tsdf._warpfield->_vertices[k], tsdf._warpfield->_transMatVec[k],new_pt);
					tsdf.TransformVertices(tsdf._warpfield->_normals[k], tsdf._warpfield->_transMatVec[k], new_normal);
					pt_clouds_trans.push_back(new_pt);
					pt_normals_trans.push_back(new_normal);
				}
				tsdf.SavePintsWithNormals(pt_clouds_trans, pt_normals_trans, outPutPathwf2);

				std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd" + std::to_string(i)+".ply";
				tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath3);

				std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal" + std::to_string(i) + ".ply";
				tsdf.SaveMeshWithNormal(foutpath);
				ptcloud1->clear();
				for (int i = 0; i < ptcloud_live->size(); ++i)
				{
					ptcloud1->push_back(ptcloud_live->points[i]);
				}
				delete[] depth_live;
			}	
		}
		tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
		std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd.ply";
		tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath3);
		
		std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal.ply";
		tsdf.SaveMeshWithNormal(foutpath);
	}
}
void RigidFusiontestMultiInputMM()
{
	std::string filepath = "E:\\kinfu\\data\\testdata\\boxing";
	std::vector<std::string> pathSet;
	getFiles(filepath, pathSet);

	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud2(new pcl::PointCloud<pcl::PointXYZ>);
	DenseICP dicp;
	//tsdfBound bound = { -128 * 5 ,128 * 5, -128 * 5, 128 * 5, 500, 500 + 256 * 5 };
	tsdfBound bound = { -64 * 5 ,64 * 5, -64 * 5, 64 * 5, 400, 400 + 128 * 5 };
	CameraIntr c1 = SetCameraIntrf2();
	int image_width = 640;
	int image_height = 480;
	float depth_scale = 1.0;
	float* depth1 = new float[image_height*image_width];
	loadDepthImages(pathSet[0], depth1, depth_scale);
	dicp.GetDensePtCloud(ptcloud1, bound, image_width, image_height, depth1, c1);
	std::string outPutPathwf00 = "E:\\kinfu\\data\\live00.ply";
	pcl::io::savePLYFile(outPutPathwf00, *ptcloud1);
	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//kdtree.setInputCloud(ptcloud1);
	//std::vector<int> pointIdxNKNSearch;
	//std::vector<float> pointNKNSquaredDistance;
	//TSDF 
	TSDF tsdf(128, 128, 128, 5, 400);
	//TSDF tsdf(256, 256, 256, 5.0);
	tsdf._knn = 5;
	CameraIntr Camera1 = SetCameraIntrf2();
	tsdf.SetCameraIntr(Camera1);
	tsdf.CalculateTruncatedSignedValue(image_width, image_height, depth1);
	tsdf.ReconWithMarchingCubes();
	std::string meshoutPutPath00 = "E:\\kinfu\\data\\tsdf_mesh_normal_00.ply";
	tsdf.SaveMeshWithNormal(meshoutPutPath00);
	if (tsdf._marchingCubesTool.IsSurfaceValid())
	{
		std::vector<Eigen::Vector3f> pt_clouds;
		std::vector<Eigen::Vector3f> pt_normals;
		if (tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals))
		{
			std::string outPutPath00 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_00.ply";
			
			std::vector<Eigen::Vector3f> filter_pt_clouds;
			std::vector<Eigen::Vector3f> filter_pt_normals;
			tsdf._warpfield->FilterNoisePointsWithRange(pt_clouds, pt_normals, 10, 25, filter_pt_clouds, filter_pt_normals);
			tsdf.SavePintsWithNormals(filter_pt_clouds, filter_pt_clouds, outPutPath00);
			tsdf._warpfield = new Warpfield(pt_clouds, pt_normals, 25);
			//tsdf._warpfield->FilterWarpField();
			std::cout << "warpfield point numbers" << tsdf._warpfield->_vertices.size() << std::endl;
			std::string outPutPathwf = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf0.ply";
			tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutPathwf);

			tsdf.AssignTSDFWeight();
			for (int i = 1; i < pathSet.size(); i++)
			{

				float *depth_live = new float[image_height*image_width];
				loadDepthImages(pathSet[i], depth_live, depth_scale);
				pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud_live(new pcl::PointCloud<pcl::PointXYZ>);
				dicp.GetDensePtCloud(ptcloud_live, bound, image_width, image_height, depth_live, c1);
				std::string outLivePath = "E:\\kinfu\\data\\ptcloud_tsdf_live" + std::to_string(i) + ".ply";
				pcl::io::savePLYFile(outLivePath, *ptcloud_live);

				pcl::PointCloud<pcl::PointXYZ>::Ptr registrated_ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
				Eigen::Matrix4f tansMat;

				dicp.EstimateTransMatMM(ptcloud1, ptcloud_live, registrated_ptcloud1, tansMat);
				std::string outICPMat = "E:\\kinfu\\data\\ICPmat" + std::to_string(i) + ".txt";
				FileIO fio;
				fio.saveMatrix(tansMat.cast<double>(), outICPMat);

				pt_clouds.clear();
				pt_normals.clear();
				filter_pt_clouds.clear();
				filter_pt_normals.clear();
				tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
				tsdf._warpfield->FilterNoisePointsWithRange(pt_clouds, pt_normals, 10, 25, filter_pt_clouds, filter_pt_normals);

				tsdf._warpfield->ICPUpdateTransMat(tansMat);
				tsdf.UpdateWarpFieldWithNgSolver(image_width, image_height,
					depth_live, i, filter_pt_clouds, filter_pt_normals);
				tsdf.CalTSDFLive(image_width, image_height, depth_live);
				tsdf.TSDFFusion();
				tsdf.ReconWithMarchingCubes();

				tsdf._warpfield->ExtendingWarpField(filter_pt_clouds, filter_pt_normals, 25);
				std::string outPutPathwf = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf" + std::to_string(i) + ".ply";
				tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutPathwf);

				std::string outPutPathwf2 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf_trans" + std::to_string(i) + ".ply";
				std::vector<Eigen::Vector3f> pt_clouds_trans;
				std::vector<Eigen::Vector3f> pt_normals_trans;
				for (int k = 0; k < tsdf._warpfield->_vertices.size(); k++)
				{
					Eigen::Vector3f new_pt;
					Eigen::Vector3f new_normal;
					if (isnan(tsdf._warpfield->_transMatVec[k](0, 0)))
					{
						std::cout << "start of matrix tsdf._warpfield->_transMatVec[k] " << k << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](0, 0) << ' ' << tsdf._warpfield->_transMatVec[k](0, 1) << ' ' << tsdf._warpfield->_transMatVec[k](0, 2) << ' ' << tsdf._warpfield->_transMatVec[k](0, 3) << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](1, 0) << ' ' << tsdf._warpfield->_transMatVec[k](1, 1) << ' ' << tsdf._warpfield->_transMatVec[k](1, 2) << ' ' << tsdf._warpfield->_transMatVec[k](1, 3) << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](2, 0) << ' ' << tsdf._warpfield->_transMatVec[k](2, 1) << ' ' << tsdf._warpfield->_transMatVec[k](2, 2) << ' ' << tsdf._warpfield->_transMatVec[k](2, 3) << std::endl;
						std::cout << tsdf._warpfield->_transMatVec[k](3, 0) << ' ' << tsdf._warpfield->_transMatVec[k](3, 1) << ' ' << tsdf._warpfield->_transMatVec[k](3, 2) << ' ' << tsdf._warpfield->_transMatVec[k](3, 3) << std::endl;
						std::cout << "end of matrix" << std::endl;
					}
					tsdf.TransformVertices(tsdf._warpfield->_vertices[k], tsdf._warpfield->_transMatVec[k], new_pt);
					tsdf.TransformVertices(tsdf._warpfield->_normals[k], tsdf._warpfield->_transMatVec[k], new_normal);
					pt_clouds_trans.push_back(new_pt);
					pt_normals_trans.push_back(new_normal);
				}
				tsdf.SavePintsWithNormals(pt_clouds_trans, pt_normals_trans, outPutPathwf2);

				std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd" + std::to_string(i) + ".ply";
				tsdf.SavePintsWithNormals(filter_pt_clouds, filter_pt_normals, outPutPath3);

				std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal" + std::to_string(i) + ".ply";
				tsdf.SaveMeshWithNormal(foutpath);

				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud_live_out(new pcl::PointCloud<pcl::PointXYZRGB>);
				//tsdf.ExtractLivePt(ptcloud_live_out);
				ptcloud1->clear();
				for (int i = 0; i < ptcloud_live->size(); ++i)
				{
					ptcloud1->push_back(ptcloud_live->points[i]);
				}
				//pt_clouds.clear();
				delete[] depth_live;
				//std::string outPutPath2 = "E:\\kinfu\\data\\ptcloud_tsdf_normal.ply";
				//tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath2);
				//std::string outPutWFPath = "E:\\kinfu\\data\\ptcloud_tsdf_warp.ply";
				//tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutWFPath);
			}
		}
		//tsdf.CalTSDFLive(image_width, image_height, depth2);
		tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
		std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd.ply";
		tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath3);

		std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal.ply";
		tsdf.SaveMeshWithNormal(foutpath);
	}
}
//void RigidFusiontestMultiInputICP()
//{
//	/*
//	std::string rgbdPath1 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png ";
//	std::string rgbdPath2 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.866730.png ";
//	std::string rgbdPath3 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.934244.png ";
//	std::string rgbdPath4 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.970244.png ";
//	std::string rgbdPath5 = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846648.003091.png ";
//	std::vector<std::string> pathSet;
//	pathSet.push_back(rgbdPath1);
//	pathSet.push_back(rgbdPath2);
//	pathSet.push_back(rgbdPath3);
//	pathSet.push_back(rgbdPath4);
//	pathSet.push_back(rgbdPath5);
//	*/
//	std::string filepath = "E:\\kinfu\\data\\testdata\\00";
//	std::vector<std::string> pathSet;
//	getFiles(filepath, pathSet);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud2(new pcl::PointCloud<pcl::PointXYZ>);
//	DenseICP dicp;
//	tsdfBound bound = { -128 * 5 ,128 * 5, -128 * 5, 128 * 5, 500, 500 + 256 * 5 };
//	//tsdfBound bound = { -64 * 5 ,64 * 5, -64 * 5, 64 * 5, 1200, 1200 + 128 * 5 };
//	CameraIntr c1 = SetCameraIntrf1();
//	int image_width = 640;
//	int image_height = 480;
//	float* depth1 = new float[image_height*image_width];
//	loadDepthImages(pathSet[0], depth1);
//	dicp.GetDensePtCloud(ptcloud1, bound, image_width, image_height, depth1, c1);
//	//TSDF 
//	//TSDF tsdf(128, 128, 128, 5.0,1200);
//	TSDF tsdf(256, 256, 256, 5.0);
//	tsdf._knn = 5;
//	CameraIntr Camera1 = SetCameraIntrf1();
//	tsdf.SetCameraIntr(Camera1);
//	tsdf.CalculateTruncatedSignedValue(image_width, image_height, depth1);
//	tsdf.ReconWithMarchingCubes();
//	if (tsdf._marchingCubesTool.IsSurfaceValid())
//	{
//		std::vector<Eigen::Vector3f> pt_clouds;
//		std::vector<Eigen::Vector3f> pt_normals;
//		if (tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals))
//		{
//			tsdf._warpfield = new Warpfield(pt_clouds, pt_normals, 25.0);
//
//			std::string outPutPathwf = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf0.ply";
//			tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutPathwf);
//
//			tsdf.AssignTSDFWeight();
//			for (int i = 1; i < pathSet.size(); i++)
//			{
//				float *depth_live = new float[image_height*image_width];
//				loadDepthImages(pathSet[i], depth_live);
//				pcl::PointCloud<pcl::PointXYZ>::Ptr ptcloud_live(new pcl::PointCloud<pcl::PointXYZ>);
//				dicp.GetDensePtCloud(ptcloud_live, bound, image_width, image_height, depth_live, c1);
//				std::string outLivePath = "E:\\kinfu\\data\\ptcloud_tsdf_live" + std::to_string(i) + ".ply";
//				pcl::io::savePLYFile(outLivePath, *ptcloud_live);
//
//				pcl::PointCloud<pcl::PointXYZ>::Ptr registrated_ptcloud1(new pcl::PointCloud<pcl::PointXYZ>);
//				Eigen::Matrix4f tansMat;
//
//				dicp.EstimateTransMat(ptcloud1, ptcloud_live, registrated_ptcloud1, tansMat);
//				std::string outICPMat = "E:\\kinfu\\data\\ICPmat" + std::to_string(i) + ".txt";
//				FileIO fio;
//				fio.saveMatrix(tansMat.cast<double>(), outICPMat);
//				tsdf._warpfield->ICPUpdateTransMatReborn(tansMat);
//				tsdf.UpdateWarpFieldWithNgSolver(image_width, image_height, depth_live, i);
//				tsdf.CalTSDFLive(image_width, image_height, depth_live);
//				tsdf.TSDFFusion();
//				tsdf.ReconWithMarchingCubes();
//				pt_clouds.clear();
//				pt_normals.clear();
//				tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
//				tsdf._warpfield->ExtendingWarpField(pt_clouds, pt_normals, 25);
//
//				std::string outPutPathwf = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf" + std::to_string(i) + ".ply";
//				tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutPathwf);
//
//				std::string outPutPathwf2 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_wf_trans" + std::to_string(i) + ".ply";
//				std::vector<Eigen::Vector3f> pt_clouds_trans;
//				std::vector<Eigen::Vector3f> pt_normals_trans;
//				for (int k = 0; k < tsdf._warpfield->_vertices.size(); k++)
//				{
//					Eigen::Vector3f new_pt;
//					Eigen::Vector3f new_normal;
//					if (isnan(tsdf._warpfield->_transMatVec[k](0, 0)))
//					{
//						std::cout << "start of matrix tsdf._warpfield->_transMatVec[k] " << k << std::endl;
//						std::cout << tsdf._warpfield->_transMatVec[k](0, 0) << ' ' << tsdf._warpfield->_transMatVec[k](0, 1) << ' ' << tsdf._warpfield->_transMatVec[k](0, 2) << ' ' << tsdf._warpfield->_transMatVec[k](0, 3) << std::endl;
//						std::cout << tsdf._warpfield->_transMatVec[k](1, 0) << ' ' << tsdf._warpfield->_transMatVec[k](1, 1) << ' ' << tsdf._warpfield->_transMatVec[k](1, 2) << ' ' << tsdf._warpfield->_transMatVec[k](1, 3) << std::endl;
//						std::cout << tsdf._warpfield->_transMatVec[k](2, 0) << ' ' << tsdf._warpfield->_transMatVec[k](2, 1) << ' ' << tsdf._warpfield->_transMatVec[k](2, 2) << ' ' << tsdf._warpfield->_transMatVec[k](2, 3) << std::endl;
//						std::cout << tsdf._warpfield->_transMatVec[k](3, 0) << ' ' << tsdf._warpfield->_transMatVec[k](3, 1) << ' ' << tsdf._warpfield->_transMatVec[k](3, 2) << ' ' << tsdf._warpfield->_transMatVec[k](3, 3) << std::endl;
//						std::cout << "end of matrix" << std::endl;
//					}
//					tsdf.TransformVertices(tsdf._warpfield->_vertices[k], tsdf._warpfield->_transMatVec[k], new_pt);
//					tsdf.TransformVertices(tsdf._warpfield->_normals[k], tsdf._warpfield->_transMatVec[k], new_normal);
//					pt_clouds_trans.push_back(new_pt);
//					pt_normals_trans.push_back(new_normal);
//				}
//				tsdf.SavePintsWithNormals(pt_clouds_trans, pt_normals_trans, outPutPathwf2);
//
//				std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd" + std::to_string(i) + ".ply";
//				tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath3);
//
//				std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal" + std::to_string(i) + ".ply";
//				tsdf.SaveMeshWithNormal(foutpath);
//
//				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcloud_live_out(new pcl::PointCloud<pcl::PointXYZRGB>);
//				//tsdf.ExtractLivePt(ptcloud_live_out);
//				ptcloud1->clear();
//				for (int i = 0; i < pt_clouds.size(); ++i)
//				{
//					pcl::PointXYZ newpt(pt_clouds[i](0), pt_clouds[i](1), pt_clouds[i](2));
//					ptcloud1->push_back(newpt);
//				}
//				//pt_clouds.clear();
//				delete[] depth_live;
//				//std::string outPutPath2 = "E:\\kinfu\\data\\ptcloud_tsdf_normal.ply";
//				//tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath2);
//				//std::string outPutWFPath = "E:\\kinfu\\data\\ptcloud_tsdf_warp.ply";
//				//tsdf.SavePintsWithNormals(tsdf._warpfield->_vertices, tsdf._warpfield->_normals, outPutWFPath);
//			}
//		}
//		//tsdf.CalTSDFLive(image_width, image_height, depth2);
//		tsdf.ExtractPointsWithNormalsFromMesh(pt_clouds, pt_normals);
//		std::string outPutPath3 = "E:\\kinfu\\data\\ptcloud_tsdf_normal_2rd.ply";
//		tsdf.SavePintsWithNormals(pt_clouds, pt_normals, outPutPath3);
//
//		std::string foutpath = "E:\\kinfu\\data\\fusionmesh_normal.ply";
//		tsdf.SaveMeshWithNormal(foutpath);
//	}
//}
void testDualQuaternionInterpolation()
{
	Eigen::Matrix4d transMat;
	transMat << 0.999981, -0.00153452, 0.00633969, -4.2154,
		0.00149396 ,0.99998, 0.00649851, 3.83545,
		-0.00634949, -0.00648897, 0.99996, 4.14248,
		0.0, 0.0, 0.0 ,1.0 ;

	TSDF tsdf;
	Eigen::Vector3f inpt;
	inpt(0) = 1000;
	inpt(1) = 1050;
	inpt(2) = 1060;
	Eigen::Vector3f outpt;
	tsdf.TransformVertices(inpt, transMat.cast<float>(), outpt);

	Dual_quat<double> dq(transMat);
	Dual_quat<double> dq1(transMat);
	Dual_quat<double> dq2 (transMat);
	Dual_quat<double> dq3(transMat);
	Dual_quat<double> dq4(transMat);
	dq = dq + dq1*0.03;
	dq = dq + dq2*0.33;
	dq = dq + dq3*0.13;
	dq = dq + dq4*0.3;
	dq.Normalize();
	transMat = dq.GetTransMat4d();
	Eigen::Vector3f outpt2;
	tsdf.TransformVertices(inpt, transMat.cast<float>(), outpt2);
	Eigen::Vector3f deltPt = outpt - outpt2;
	std::cout << "delt after transformed"<<deltPt(0) << " " << deltPt(1) << " " << deltPt(2) << std::endl;
	std::cout << "delt after transformed0" << outpt(0) << " " << outpt(1) << " " << outpt(2) << std::endl;
	std::cout << "delt after transformed2" << outpt2(0) << " " << outpt2(1) << " " << outpt2(2) << std::endl;
	std::cout << transMat(0, 0) << " " << transMat(0, 1) << " " << transMat(0, 2) << " " << transMat(0, 3) << " "
		<< transMat(1, 0) << " " << transMat(1, 1) << " " << transMat(1, 2) << " " << transMat(1, 3) << " "
		<< transMat(2, 0) << " " << transMat(2, 1) << " " << transMat(2, 2) << " " << transMat(2, 3) << " "
		<< transMat(3, 0) << " " << transMat(3, 1) << " " << transMat(3, 2) << " " << transMat(3, 3) << std::endl;
}

int main()
{
	//std::string path = "E:\\kinfu\\data\\rgbd_dataset_freiburg3_walking_rpy\\depth\\1341846647.802269.png";//data\\1341846647.802269.png";
	//std::string path = "C:\\Users\\sensetime\\Downloads\\1.png";
	//loadDepthImages(path);
	//tsdftest();
	//ICPtest();
	RigidFusiontestMultiInputMM();
	//RigidFusiontestMultiInput();
	//testDualQuaternionInterpolation();
	//RigidFusiontest();
	//testMC();
	//testConvertImages();
	return 0;
}

