#include "Se3.hpp"

template<class T> Se3<T>::Se3(){}   
template<class T> Se3<T>::~Se3(){  };

template<class T> Se3<T>::Se3(T value[6])
{
    for(int i =0; i<6; i++)
    {
        this->_value(i,0) = value[i];
    }
	init();
}
template<class T> Se3<T>::Se3(const Eigen::Matrix<T,6,1> &value)
{
    this->_value = value;
	init();
}
template<class T> Se3<T>::Se3(const Eigen::Matrix<T,3,1> &so3,const Eigen::Matrix<T,3,1> &v)
{
    this->_value(0,0) = so3(0);
    this->_value(1,0) = so3(1);
    this->_value(2,0) = so3(2);

    this->_value(3,0) = v(0);
    this->_value(4,0) = v(1);
    this->_value(5,0) = v(2);
	init();
}

template<class T> void VectorToSkewSyMat(const Eigen::Matrix<T,3,1> &w, Eigen::Matrix<T,3,3> &skewMat)
{
    skewMat(0,0) =T(0);
    skewMat(0,1) = -w(2);
    skewMat(0,2) = w(1);

    skewMat(1,0) = w(2);
    skewMat(1,1) = 0;
    skewMat(1,2) = -w(0);

    skewMat(2,0) = -w(1);
    skewMat(2,1) = w(0);
    skewMat(2,2) = T(0);
}
template<class T> void Se3<T>::init()
{
    this->_norm = this->_value.norm();
	
    this->_so3(0) = this->_value(0,0); 
    this->_so3(1) = this->_value(1,0); 
    this->_so3(2) = this->_value(2,0); 

    this->_v(0) = this->_value(3,0); 
    this->_v(1) = this->_value(4,0); 
    this->_v(2) = this->_value(5,0);
	this->_norm_w = this->_so3.norm();
	if (this->_norm_w == 0)
	{
		this->_SE3.setIdentity();
		this->_SE3.block(0, 3, 3, 1) = this->_v;
	}
	else 
	{
		VectorToSkewSyMat(this->_so3, this->_skewsymMat);
		this->_SE3.setIdentity();
		this->UpdateSE3();
	}
   
}
template<class T> void Se3<T>::update()
{
    this->init();
}
template<class T> void Se3<T>::expMapSo3()
{
    Eigen::Matrix<T,3,3> identityMat;
    identityMat.setIdentity();
    this->_SO3 = identityMat + (sin(this->_norm_w)/this->_norm_w)* this->_skewsymMat
    + ((1-cos(this->_norm_w))/(this->_norm_w * this->_norm_w))* this->_skewsymMat * this->_skewsymMat;
}
template<class T> void Se3<T>::UpdateSE3()
{
	//this->_SE3.setIdentity();
    this->expMapSo3();
    this->_SE3.block(0,0,3,3) = this->_SO3;
    /*
    for(int i =0; i< 3; i++)
    {
        for(int j =0; j< 3; j++)
        {
            this->_SE3(i,j) = this->_SO3(i,j);
        }
    }*/
    Eigen::Matrix<T,3,3> identityMat;
    identityMat.setIdentity();
    Eigen::Matrix<T,3,3> V = identityMat + ((1-cos(this->_norm_w))/this->_so3.squaredNorm())*this->_skewsymMat
    + (this->_norm_w - sin(this->_norm_w))/(this->_norm_w * this->_so3.squaredNorm()) * this->_skewsymMat * this->_skewsymMat;
    this->_offsetT = V * this->_v;
    //for(int i =0; i< 3; i++)
    //{
    //    this->_SE3(3,i) = this->_offsetT(i,0);
    //}
    this->_SE3.block(0,3,3,1) = this->_offsetT; 
}
template<class T> Eigen::Matrix<T,4,4> Se3<T>::GetTransMat4dFromSe3()
{
    return this->_SE3;
}
template<class T> T Se3<T>::CalculateSe3Norm()
{
    return this->_value.norm();
}

template class Se3<double>;
template class Se3<float>;