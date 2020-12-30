#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};

class ProjectionFactorAutoDiff {
public:
    ProjectionFactorAutoDiff(const Eigen::Vector3d& pts_i, const Eigen::Vector3d& pts_j)
            : _pts_i(pts_i), _pts_j(pts_j) {}

    template <typename T>
    bool operator()(const T* const T_w_i, const T* const T_w_j,
                    const T* const T_i_c, const T* const inv,  T* residuals_ptr) const {

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > Pi(T_w_i);
        Eigen::Map<const Eigen::Quaternion<T> > Qi(T_w_i + 3);

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > Pj(T_w_j);
        Eigen::Map<const Eigen::Quaternion<T> > Qj(T_w_j + 3);

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > tic(T_i_c);
        Eigen::Map<const Eigen::Quaternion<T> > qic(T_i_c + 3);


        T inv_dep_i = *inv;

        Eigen::Matrix<T, 3, 1> pts_camera_i = _pts_i.cast<T>() / inv_dep_i;

        Eigen::Matrix<T, 3, 1> pts_imu_i = qic * pts_camera_i + tic;

        Eigen::Matrix<T, 3, 1> pts_w = Qi * pts_imu_i + Pi;

        Eigen::Matrix<T, 3, 1> pts_imu_j = Qj.conjugate() * (pts_w - Pj);

        Eigen::Matrix<T, 3, 1> pts_camera_j = qic.conjugate() * (pts_imu_j - tic);

        Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals_ptr);

        T dep_j = pts_camera_j.z();
        residual = (pts_camera_j / dep_j).template head<2>() - _pts_j.template head<2>();

        residual.applyOnTheLeft(sqrt_information_.template cast<T>());

        return true;
    }

    static ceres::CostFunction* Create(
            const Eigen::Vector3d& pts_i, const Eigen::Vector3d& pts_j) {
        return new ceres::AutoDiffCostFunction<ProjectionFactorAutoDiff, 2, 7, 7, 7, 1>(
                new ProjectionFactorAutoDiff(pts_i, pts_j));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    Eigen::Vector3d _pts_i, _pts_j;
public:
    static Eigen::Matrix2d sqrt_information_;
};

