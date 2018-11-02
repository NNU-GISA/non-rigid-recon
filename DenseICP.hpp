#include <pcl/registration/icp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
typedef pcl::PointXYZ pointT;
struct tsdfBound
{
	float x_down_bound;
	float x_up_bound;
	float y_down_bound;
	float y_up_bound;
	float z_down_bound;
	float z_up_bound;
};
class DenseICP
{
    public:
        DenseICP(){};
        ~DenseICP(){};
		void GetDensePtCloud(pcl::PointCloud<pointT>::Ptr ptCloud, const tsdfBound &bound,
			int image_width,int image_height, float *depth,const Eigen::Matrix3f &camera_intr);
        void EstimateTransMat(pcl::PointCloud<pointT>::Ptr src_Ptcloud,
        pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
        pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
        Eigen::Matrix4f &transMat);

		void EstimateTransMatSmallStep(
			pcl::PointCloud<pointT>::Ptr src_Ptcloud,
			pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
			pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
			Eigen::Matrix4f &transMat);

		void EstimateTransMatBigStep(
			pcl::PointCloud<pointT>::Ptr src_Ptcloud,
			pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
			pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
			Eigen::Matrix4f &transMat);
		void EstimateTransMatMM(
			pcl::PointCloud<pointT>::Ptr src_Ptcloud,
			pcl::PointCloud<pointT>::Ptr tar_Ptcloud,
			pcl::PointCloud<pointT>::Ptr registrated_Ptcloud,
			Eigen::Matrix4f &transMat);

};