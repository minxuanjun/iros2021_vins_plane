//
// Created by xubo on 20-9-15.
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>


template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
{
    //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}


template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
{
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);

    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;

    ans.template block<3, 3>(0, 0) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
    ans(3, 3) = qq.w();

    ans.template block<1, 3>(3, 0) = -qq.vec().transpose();
    ans.template block<3, 1>(0, 3) = qq.vec();

    return ans;
}
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 3> Qleft_LocalParameter(const Eigen::QuaternionBase<Derived> &q)
{
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);

    Eigen::Matrix<typename Derived::Scalar, 4, 3> ans;

    ans.template block<3, 3>(0, 0) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());

    ans.template block<1, 3>(3, 0) = -qq.vec().transpose();
    ans = typename Derived::Scalar(0.5)*ans;
//    ans.template block<3, 1>(0, 3) = qq.vec();

    return ans;
}
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
{
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(p);

    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;

    ans.template block<3, 3>(0, 0) = -(qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec()));
    ans(3, 3) = qq.w();

    ans.template block<1, 3>(3, 0) = -qq.vec().transpose();
    ans.template block<3, 1>(0, 3) = qq.vec();

    return ans;
}

int main()
{

    Eigen::Quaterniond q(1, 2, 3, 4);

    q.normalize();

    Eigen::Quaterniond q_inv = q.inverse();


    std::cout << "q; " << q.coeffs() << std::endl;
    Eigen::Vector3d p{1.,2.,3};

    Eigen::Matrix3d JRJp = -q.toRotationMatrix()*skewSymmetric(p);

    Eigen::Matrix3d JRJp_inv = skewSymmetric(q_inv * p);

    std::cout << "JRJp: \n" << JRJp << std::endl;
    std::cout << "JRJp_inv \n" << JRJp_inv << std::endl;

    Eigen::Matrix<double, 3,4> JqpJq;
    Eigen::Matrix<double, 3,4> JqpJq_inv;

    Eigen::Vector3d qxyz{q.x(), q.y(), q.z()};

    Eigen::Vector3d qxyz_inv{q_inv.x(), q_inv.y(), q_inv.z()};

    JqpJq.col(3) = 2*(q.w()*p + skewSymmetric(qxyz)*p);
    JqpJq_inv.col(3) = 2*(q_inv.w()*p + skewSymmetric(qxyz_inv)*p);
    double tmp = qxyz.transpose()*p;
    double tmp2 = qxyz_inv.transpose()*p;
    JqpJq.leftCols(3) = 2*(tmp*Eigen::Matrix3d::Identity()+ qxyz*p.transpose() - p*qxyz.transpose() - q.w()*skewSymmetric(p));
    JqpJq_inv.leftCols(3) = 2*(tmp2*Eigen::Matrix3d::Identity()+ qxyz_inv*p.transpose() - p*qxyz_inv.transpose() - q_inv.w()*skewSymmetric(p));


    JqpJq = JqpJq;
    JqpJq_inv = -JqpJq_inv;
    JqpJq_inv.rightCols(1) = -JqpJq_inv.rightCols(1);

    Eigen::Matrix<double, 4,3> JqJtheta;
    JqJtheta = Qleft_LocalParameter(q);


    Eigen::Matrix3d JqJthta = JqpJq* JqJtheta;
    Eigen::Matrix3d JqJthta_inv = JqpJq_inv* JqJtheta;

    std::cout << "JqJp: \n" << JqJthta << std::endl;
    std::cout << "JqJp_inv: \n" << JqJthta_inv << std::endl;
    return 0;
}