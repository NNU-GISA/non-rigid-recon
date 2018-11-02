#ifndef SE3_H
#define SE3_H
#include <Eigen/Dense>
template<class T>
class Se3 
{
    public:
        Se3();
        ~Se3();
        Se3(T value[6]);
        Se3(const Eigen::Matrix<T,6,1> &value);
        Se3(const Eigen::Matrix<T, 3, 1> &so3,const Eigen::Matrix<T, 3, 1> &v);
        void init();
        void update();
        Eigen::Matrix<T, 4, 4> GetTransMat4dFromSe3();
    private:
        T CalculateSe3Norm();
        void UpdateSE3();
        void expMapSo3();
    public:
        Eigen::Matrix<T,6,1> _value;
        Eigen::Matrix<T, 3, 1> _so3;
        Eigen::Matrix<T, 3, 1> _v;
        
    private:
        Eigen::Matrix<T, 3, 3> _skewsymMat;
        Eigen::Matrix<T, 4, 4> _SE3;
        Eigen::Matrix<T, 3, 3> _SO3;
        Eigen::Matrix<T,3,1> _offsetT;
        T _norm;
		T _norm_w;
        //Eigen::Quaterniond newq; 
};
#endif