#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionTdFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1>
{
  public:
    ProjectionTdFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
    				   const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
    				   const double _td_i, const double _td_j, const double _row_i, const double _row_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    double row_i, row_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};



class ProjectionTdFactorAutoDiff{
public:
    ProjectionTdFactorAutoDiff(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
                               const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
                               const double _td_i, const double _td_j, const double _row_i, const double _row_j)
            : pts_i(_pts_i), pts_j(_pts_j),td_i(_td_i), td_j(_td_j)
    {
        velocity_i.x() = _velocity_i.x();
        velocity_i.y() = _velocity_i.y();
        velocity_i.z() = 0;
        velocity_j.x() = _velocity_j.x();
        velocity_j.y() = _velocity_j.y();
        velocity_j.z() = 0;
        row_i = _row_i - ROW / 2;
        row_j = _row_j - ROW / 2;
    };

    template <typename T>
    bool operator()(const T* const pose_i, const T* const pose_j,
                    const T* const T_i_c,  const T* const inv_depth,
                    const T* const td, T* residuals_ptr) const {


        Eigen::Map<const Eigen::Matrix<T, 3, 1> > Pi(pose_i);
        Eigen::Map<const Eigen::Quaternion<T> > Qi(pose_i + 3);

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > Pj(pose_j);
        Eigen::Map<const Eigen::Quaternion<T> > Qj(pose_j + 3);

        Eigen::Map<const Eigen::Matrix<T, 3, 1> > t_i_c(T_i_c);
        Eigen::Map<const Eigen::Quaternion<T> > q_i_c(T_i_c + 3);

        T inv = *inv_depth;
        T td_est = *td;


        Eigen::Matrix<T, 3, 1> pts_i_td, pts_j_td;

        pts_i_td = pts_i.template cast<T>() - (td_est - static_cast<T>(td_i) + static_cast<T>(TR) / static_cast<T>(ROW)
                                                                            * static_cast<T>(row_i)) * velocity_i.template cast<T>();
        pts_j_td = pts_j.template cast<T>() - (td_est - static_cast<T>(td_j) + static_cast<T>(TR) / static_cast<T>(ROW) *
                                                                              static_cast<T>(row_j)) * velocity_j.template cast<T>();
        Eigen::Matrix<T, 3, 1> pts_camera_i = pts_i_td / inv;
        Eigen::Matrix<T, 3, 1> pts_imu_i = q_i_c * pts_camera_i + t_i_c;
        Eigen::Matrix<T, 3, 1> pts_w = Qi * pts_imu_i + Pi;
        Eigen::Matrix<T, 3, 1> pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Eigen::Matrix<T, 3, 1> pts_camera_j = q_i_c.conjugate() * (pts_imu_j - t_i_c);


        Eigen::Map<Eigen::Matrix<T, 2, 1>> residual(residuals_ptr);

        T dep_j = pts_camera_j.z();
        residual = (pts_camera_j / dep_j).template head<2>() - pts_j_td.template head<2>();


        residual.applyOnTheLeft(sqrt_info.template cast<T>());


        return true;
    }

    static ceres::CostFunction* Create(
            const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
            const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
            const double _td_i, const double _td_j, const double _row_i, const double _row_j) {
        return new ceres::AutoDiffCostFunction<ProjectionTdFactorAutoDiff, 2, 7, 7, 7, 1, 1>(
                new ProjectionTdFactorAutoDiff(_pts_i, _pts_j, _velocity_i, _velocity_j,_td_i, _td_j, _row_i, _row_j));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    double row_i, row_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;

};
