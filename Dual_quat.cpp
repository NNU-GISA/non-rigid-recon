#include "Dual_quat.h"
//#include <pcl/common/transforms.h>
//using namespace pcl;

template<class T> Dual_quat<T>::Dual_quat()
{
    Quaternion<T> new_qr(T(0.0),T(0.0),T(0.0),T(0.0));
    Quaternion<T> new_qt(T(0.0),T(0.0),T(0.0),T(0.0));
    this->_RQ = new_qr;
    this->_TQ = new_qt;
    this->_trans(0,0) = T(0.0);
    this->_trans(1,0) = T(0.0);
    this->_trans(2,0) = T(0.0);

};
template<class T> Dual_quat<T>::Dual_quat(const Matrix<T, 4, 4> &transMat)
{
	this->_pclTransMat = transMat;
	Matrix<T, 3, 3> rotationMat = transMat.block(0,0,3,3);
	Matrix<T, 3, 1> translationMat = transMat.block(0,3,3,1);
	Quaternion<T> RQ(rotationMat);
	init(RQ, translationMat);
}
template<class T> void Dual_quat<T>::init(const Quaternion<T> &RQ,const Matrix<T,3,1> &trans)
{
    this->_RQ = RQ;
    this->_trans = trans;
    Quaternion<T> new_qt(T(0.0),trans(0),trans(1),trans(2));
    this->_TQ = new_qt;
    this->init();
};
template<class T> Dual_quat<T>::~Dual_quat(){};

template<class T> void Dual_quat<T>::init()
{
    this->_qr = this->_RQ;
    this->_qd = this->_TQ * this->_RQ;
    //Quaterniond new_q(0.5,0.5,0.5,0.5);
    this->_qd = QuaterniondMultiply(this->_qd,T(0.5));
}

template<class T> Eigen::Matrix<T,4,4> Dual_quat<T>::GetTransMat4d()
{
	Eigen::Matrix<T, 3, 3> rotationMat= this->_qr.normalized().toRotationMatrix();
    this->_TQ = this->GetTQ();
    this->_trans(0) = T(this->_TQ.x());
    this->_trans(1) = this->_TQ.y();
    this->_trans(2) = this->_TQ.z();
    this->AssignTransMat4d(rotationMat,this->_trans);
    return this->_pclTransMat;
};

template<class T> Eigen::Quaternion<T> Dual_quat<T>::GetRQ()
{
    this->_RQ = this->_qr;
    return this->_RQ;
}

template<class T> Eigen::Quaternion<T> Dual_quat<T>::GetTQ()
{
    this->_TQ = this->_qd * this->_qr.conjugate();
    this->_TQ = QuaterniondMultiply(this->_TQ,T(2.0)) ;
    return this->_TQ;
}

template<class T> void Dual_quat<T>::AssignTransMat4d( const Eigen::Matrix<T, 3, 3> &rotationMat,
	const Matrix<T,3,1> &trans)
{
	this->_pclTransMat.setZero();
    for(int i =0; i<3; i++)
    {
        for(int j =0; j < 3; j++)
        {
            this->_pclTransMat(i,j) = rotationMat(i,j);
        }
    }
    this->_pclTransMat(0,3) =  trans(0);
    this->_pclTransMat(1,3) =  trans(1);
    this->_pclTransMat(2,3) =  trans(2);
    this->_pclTransMat(3,3) = T( 1.0);
};

template<class T> Eigen::Quaternion<T> QuaterniondAdd(Quaternion<T> q1, Quaternion<T> q2)
{
    Quaternion<T> new_q(
        q1.w() + q2.w(),
        q1.x() + q2.x(),
        q1.y() + q2.y(),
        q1.z() + q2.z());
    return new_q;
}   

template<class T> Eigen::Quaternion<T> QuaterniondMultiply(Quaternion<T> q1, T weight)
{
    Quaternion<T> new_q(
        q1.w() * weight,
        q1.x() * weight,
        q1.y() * weight,
        q1.z() * weight
    );
    return new_q;
}

template<class T> Dual_quat<T> Dual_quat<T>::operator+ (const Dual_quat<T> &dq1)
{
    Dual_quat<T> dq2;
    dq2._qr = QuaterniondAdd(this->_qr, dq1._qr);
    dq2._qd = QuaterniondAdd(this->_qd, dq1._qd);
    /*
    dq2._qr.w = this->_qr.w + dq1._qr.w;
    dq2._qr.x = this->_qr.x + dq1._qr.x;
    dq2._qr.y = this->_qr.y + dq1._qr.y;
    dq2._qr.z = this->_qr.z + dq1._qr.z;
    
    dq2._qd.w = this->_qd.w + dq1._qd.w;
    dq2._qd.x = this->_qd.x + dq1._qd.x;
    dq2._qd.y = this->_qd.y + dq1._qd.y;
    dq2._qd.z = this->_qd.z + dq1._qd.z;
    */
    return dq2;
}
template<class T> Dual_quat<T> Dual_quat<T>::operator* (const T factor)
{
    Dual_quat<T> dq2;
    dq2._qr = QuaterniondMultiply(this->_qr, factor);
    dq2._qd = QuaterniondMultiply(this->_qd, factor);
    /*
    dq2.w = this->_RQ.w *factor;
    RQ2.x = this->_RQ.x *factor;
    RQ2.y = this->_RQ.y *factor;
    RQ2.z = this->_RQ.z *factor;
    trans2 = this->_trans *factor;
    Dual_quat dq2(RQ2,trans2);
    */
    return dq2;
}

template<class T> Dual_quat<T> Dual_quat<T>::operator* (const Dual_quat<T> &dq1)
{
    Dual_quat<T> new_dq;
    new_dq._qr = this->_qr* dq1._qr;
    new_dq._qd = QuaterniondAdd(this->_qr * dq1._qd, this->_qd * dq1._qr);
    return new_dq;
}

template<class T> Dual_quat<T> Dual_quat<T>::Getconjugate() const
{
    Dual_quat<T> dq_new;
    dq_new._qr = this->_qr.conjugate();
    dq_new._qd = this->_qd.conjugate();
    return dq_new;
}

template<class T> void Dual_quat<T>::Normalize()
{
    //Quaternion<T> dqr_conjugate = this->_qr * this->_qr.conjugate() ;
    //Dual_quat<T> dq_mult = (*this) * dq_conjugate;
    //double qrSqNorm = this->_qr.squaredNorm();
    //double qdSqNorm = this->_qd.squaredNorm();
	T sqNorm = this->_qr.squaredNorm(); //+ dq_mult._qd.squaredNorm();
    sqNorm = sqrt(sqNorm);
    *this = (*this) * (T(1/sqNorm));
};
/*
void Dual_quat::TranformVertex(pcl::PointXYZLNormal &srcPt, pcl::PointXYZLNormal &tarPt)
{
    Eigen::Matrix<double, 3, 1> offset;
    offset(0,0) = this->_trans(0);
    offset(1,0) = this->_trans(0);
    offset(2,0) = this->_trans(0);

    pcl::transformPointCloudWithNormals(srcPt,tarPt,offset,this->GetRQ());
}*/
/*
template<template T> void Dual_quat<T>::TranformPt_cloud(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr ptSrc_cloud,
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr ptTar_cloud)
{
    Eigen::Matrix<T, 3, 1> offset;
    offset(0,0) = this->_trans(0);
    offset(1,0) = this->_trans(0);
    offset(2,0) = this->_trans(0);
    pcl::transformPointCloudWithNormals(*ptSrc_cloud, *ptTar_cloud, offset,this->GetRQ());
}*/
template class Dual_quat<double>;
template class Dual_quat<float>;