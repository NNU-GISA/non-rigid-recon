#include "FileIO.h"
#include <fstream>


FileIO::FileIO()
{
}


FileIO::~FileIO()
{
}

void FileIO::saveMatrix(const Eigen::MatrixXd &mat, const std::string &file)
{
	std::ofstream ofile;
	ofile.open(file);
	int rows = mat.rows();
	int cols = mat.cols();
	std::string fileBuffer;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			ofile << mat(i, j) << " ";
		}
		ofile << std::endl;
	}
	ofile.close();
}

const void FileIO::SavePintsWithNormals(const std::vector<Eigen::Vector3f>&pt_clouds,
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

	filestream << "element face 0" << std::endl;
	filestream << "property list uchar int vertex_indices" << std::endl;
	filestream << "end_header" << std::endl;

	for (int i = 0; i < point_number; i++)
	{
		filestream << pt_clouds[i](0) << " " << pt_clouds[i](1) << " " << pt_clouds[i](2) << " "
			<< pt_normals[i](0) << " " << pt_normals[i](1) << " " << pt_normals[i](2) << std::endl;
	}

}