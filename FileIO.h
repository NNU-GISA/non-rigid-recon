#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>
class FileIO
{
public:
	FileIO();
	~FileIO();
	void saveMeshPly(const std::string &outPath);
	void saveMatrix(const Eigen::MatrixXd &mat, const std::string &file);
	const void SavePintsWithNormals(const std::vector<Eigen::Vector3f>&pt_clouds,
		const std::vector<Eigen::Vector3f>&pt_normals, const std::string &path);
};

