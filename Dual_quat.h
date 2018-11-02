#ifndef DUAL_QUAT_H
#define DUAL_QUAT_H
#include <Eigen/Dense>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
using namespace Eigen;
//using namespace pcl;
template<class T>
class Dual_quat
{
    public:
        Dual_quat();
		Dual_quat(const Eigen::Matrix<T, 4, 4> &transMat);
		//Dual_quat();
        void init(const Quaternion<T> &RQ,const Matrix<T,3,1> &trans);
        ~Dual_quat();
        void init();
        Eigen::Matrix<T,4,4> GetTransMat4d();
        Eigen::Quaternion<T> GetRQ();
        Eigen::Quaternion<T> GetTQ();
       // Eigen::Quaterniond QuaterniondAdd(Quaterniond q1, Quaterniond q2);
       // Eigen::Quaterniond QuaterniondMultiply(Quaterniond q1, double q2);

        void AssignTransMat4d( const Eigen::Matrix<T,3,3> &rotationMat,const Matrix<T,3,1> &trans);

        Dual_quat<T>  operator+ (const Dual_quat<T> &dq1);
        Dual_quat<T> operator* (const T factor);
        Dual_quat<T> operator* (const Dual_quat<T> &dq1);

        Dual_quat<T> Getconjugate() const;
    
        void Normalize();
    //public:
        //void TranformVertex(pcl::PointXYZLNormal &srcPt, pcl::PointXYZLNormal &tarPt);
        //void TranformPt_cloud( pcl::PointCloud<pcl::PointXYZLNormal>::Ptr ptSrc_cloud,
       // pcl::PointCloud<pcl::PointXYZLNormal>::Ptr ptTar_cloud);

    public: 
        //dual quaternion real and virtual
        Eigen::Quaternion<T> _qr;
        Eigen::Quaternion<T> _qd;

    private:
        Eigen::Matrix<T,4,4> _pclTransMat;
        Eigen::Matrix<T,3,1> _trans;
        Eigen::Quaternion<T> _RQ;
        Eigen::Quaternion<T> _TQ;
};
template<class T>
Eigen::Quaternion<T> QuaterniondAdd(Quaternion<T> q1, Quaternion<T> q2);
template<class T>
Eigen::Quaternion<T> QuaterniondMultiply(Quaternion<T> q1, T q2);

#endif