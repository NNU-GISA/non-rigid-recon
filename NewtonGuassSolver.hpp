#include <Eigen/Dense>
#include "Warpfield.hpp"
//#include "Dual_quat.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include "Se3.hpp"
#include <Eigen/Sparse>
using namespace Eigen;
using namespace std;


class NewtonGuassSolver
{
    public:
        NewtonGuassSolver();
		NewtonGuassSolver(const Eigen::Matrix3f &cameraIntr,
			int image_width, int image_height, float* depth);
        ~NewtonGuassSolver();
            
    public:
		void SetCameraIntr(const Eigen::Matrix3f &cameraIntr);
		void SetDepthInfo(int image_width, int image_height, float* depth);
		void FindCorrespondenceCanonical(const float3Vec &inPt_cloud,
			const float3Vec &inPt_normals,
			float3Vec &outPt_cloud, float3Vec &outPt_normals,
			float3Vec &corPt_cloud, float radius);
		void FindCorrespondence(const float3Vec &inPt_cloud, const float3Vec &inPt_normals,
			 const std::vector<float> &inweightVec,
			 const std::vector< Eigen::Matrix4f> &in_transMatVec,
			 float3Vec &outPt_cloud, float3Vec &outPt_normals,
			 std::vector<float> &weightVec,
			 std::vector< Eigen::Matrix4f> &transMatVec,
			 float3Vec &corPt_cloud);

		void CalculateDataEnergy(const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const std::vector< Eigen::Matrix4f> &transMatVec,
			const float3Vec &corPt_cloud, VectorXd &dataEngergyMat);
		//cal all data association pts energy 
		void CalculateDataEnergyNew(const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const float3Vec &corPt_cloud, VectorXd &dataEngergyMat, float radius);
		//for each warpfield point, calculate data loss
		double CalDataSingleTerm(const Eigen::Vector3f &inpt,
			const Eigen::Vector3f &normal,
			const Eigen::Matrix4f &transMat,
			const Eigen::Vector3f &corpt);

		void CalDerivativeNumerical(const Eigen::Vector3f &inpt,
			const Eigen::Vector3f &normal,
			const Eigen::Matrix4f &transMat,
			const Eigen::Vector3f &corpt,
			Eigen::Matrix<double,6,1> &derivativeVec);
		void CalDerivativeNumericalNew(const Eigen::Vector3f &inpt,
			const Eigen::Vector3f &normal,
			const int wfIndex,float radius,
			const Eigen::Vector3f &corpt,
			Eigen::Matrix<double, 6, 1> &derivativeVec);
		void CalculateJacobiDataTermAnalytic(const Eigen::Vector3d &inpt,
			const Eigen::Vector3d &normal,
			const Eigen::Matrix4d &transMat,
			const Eigen::Vector3d &corpt,
			Eigen::Matrix<double, 1, 6> &derivativeVec);
		void CalculateJacobiDataTermAnalyticNew(const Eigen::Vector3d &inpt,
			const Eigen::Vector3d &normal,
			const Eigen::Matrix4d &transMat,
			const Eigen::Vector3d &corpt,
			Eigen::Matrix<double, 1, 6> &derivativeVec);
		void BuildNewWarpfield(const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const std::vector<float> &weightVec,
			const std::vector< Eigen::Matrix4f> &transMatVec, float dgw);
		void BuildDeformationTree(float epsion);

		void CalculateJacobiMatDataTerm(const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const std::vector< Eigen::Matrix4f> &transMatVec, const float3Vec &corPt_cloud,
			MatrixXd &jacobiMat);
		//this is for sparse matrix
		void CalculateJacobiMatDataTerm(const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const std::vector< Eigen::Matrix4f> &transMatVec, const float3Vec &corPt_cloud,
			SparseMatrix<double> &jacobiMat);
		void CalculateJacobiMatDataTermNumerical(const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const std::vector< Eigen::Matrix4f> &transMatVec, const float3Vec &corPt_cloud,
			SparseMatrix<double> &jacobiMat);
		void CalculateJacobiMatDataTermNumericalNew(
			const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const float3Vec &corPt_cloud, float radius,
			SparseMatrix<double> &jacobiMat);
		void CalculateJacobiMatDataTermAnalyticalNew(
			const float3Vec &pt_cloud, const float3Vec &pt_normals,
			const float3Vec &corPt_cloud, float radius,
			SparseMatrix<double> &jacobiMat);
		double CalRegPenaltySingleTerm(const Eigen::Vector3f &inpt,const Eigen::Matrix4f &transMat, float weight);
		
		void CalRegularizationPenalty(const float3Vec &pt_cloud,
			const std::vector< Eigen::Matrix4f> &transMatVec,
			const std::vector<float> &weightVec,
			VectorXd &dataEngergyMat);
		
		void CalJacobiMatReg(const float3Vec &pt_cloud,
			const std::vector< Eigen::Matrix4f> &transMatVec,
			const std::vector<float> &weightVec, MatrixXd &jacobiRegMat);

		void CalJacobiMatReg(const float3Vec &pt_cloud,
			const std::vector< Eigen::Matrix4f> &transMatVec,
			const std::vector<float> &weightVec, SparseMatrix<double> &jacobiRegMat);
        // void FindCorrespendence(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr input_cloud,
        // pcl::PointCloud<pcl::PointXYZLNormal>::Ptr livept_cloud,vector<int> &correspondIndex);

		void NewtonGaussOptimizer(Warpfield **wf, int iterationCount =0);
		void NewtonGaussOptimizer(Warpfield &wf);
		void NewtonGaussOptimizer2(Warpfield **wf, int iterationCount,
			const float3Vec &mcPts, const float3Vec &mcNormals);
	private:
		void NGOptimizer(Warpfield **wf, int iterationCount,
			float3Vec &corPt_cloud);
		void NGOptimizerNew(const float3Vec &cananicalPts,
			const float3Vec &cananicalNormals,
			int iterationCount, float radius,
			const float3Vec &corPt_cloud);
		void NGOptimizerNewNumerical(const float3Vec &cananicalPts,
			const float3Vec &cananicalNormals,
			int iterationCount, float radius, float rate,
			const float3Vec &corPt_cloud);
		void NGOptimizerTest(Warpfield **wf, int iterationCount,
			float3Vec &corPt_cloud);
		/*
        double CalculateEnergyFunc(int knnNum, 
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfield,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud_live);

		
        //void initNGSolver(const pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud);
        double DerivativeNumerical(const pcl::PointXYZLNormal canonical_pt, Warpfield _wf, const pcl::PointXYZLNormal live_pt);
            
        //robust turkey penalty
        inline double CalDataPenalty(const Vector3d &vertex_u,
            const Vector3d &vertex_l, const Vector3d &normal_u);
        //Huber penalty
        double CalRegPenalty(const Warpfield& N_warpi,const Warpfield& N_warpj, const float fiReg);
        void GetInitTransMatViaICP();

        void Optimizer(); 

        void CalculateJacobiMat_dataTermAndEnergy(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfieldVector,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud_live,
        Eigen::MatrixXd &JacMat,  Eigen::MatrixXd &EnergyMat,int pt_size);

        //k is k-nearest number
        void CalculateJacobiMat_regTermAndEnergy(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfieldVector,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr hierarchicalpt_cloud,
        std::vector<Warpfield> &hierarchicalGraphWFV,
        int k,
        Eigen::MatrixXd &JacMat, Eigen::MatrixXd &EnergyMat,int pt_size);
        /*
        void CalculateHessianAndJacMat(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfieldVector,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud_live,
        Eigen::MatrixXd &HessianMat, Eigen::MatrixXd &JacMat);*/
		/*
        void CalculateHessianAndJacMatAndEnergy(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfieldVector,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud_live,Eigen::MatrixXd &HessianMat,
        Eigen::MatrixXd &JacMat, Eigen::MatrixXd &EnergyMat,int lambuda = 200);

        void EvaluateEnergy(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfieldVector,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud_live,
        Eigen::MatrixXd &EnergyMat);

        void CalculateDeltSe3(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud, 
        std::vector<Warpfield> &warpfieldVector,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr pt_cloud_live,
        Eigen::MatrixXd &deltse3Mat); */


    public:
        vector<Se3<float>> _se3Set;
		Eigen::Matrix3f _cameraIntr;
		float _visibilityLimit = 50;
		std::vector<int> _warpField_indexVec;
		Warpfield* _wf;
		std::vector<int> _defTreePtIndexVec;
		Warpfield* _defromationTree;
	private:
		int _image_width;
		int _image_height;
		float* _depth;

		//
		Eigen::MatrixXi _visiblityMap;

        //pcl::PointXYZ newpt;
};

