//
// Created by ubuntu on 2021/1/2.
//

#pragma once

// ceres
#include <ceres/ceres.h>

// self
#include "../utils/EigenTypes.h"
#include "../utils/Twist.h"
#include "../utils/math_utils.h"

class ProjectionPlaneFactorAutoDiff {
 public:
  ProjectionPlaneFactorAutoDiff(const Eigen::Vector3d& pts_i,
                                const Eigen::Vector3d& pts_j)
      : pts_i_(pts_i), pts_j_(pts_j) {}

  template <typename T>
  Eigen::Matrix<T, 4, 1> TransformPlane(
      const Eigen::Matrix<T, 4, 1>& PlaneCoeff,
      const Twist<T>& T_to_from) const {
    using Vector4T = Eigen::Matrix<T, 4, 1>;
    using Matrix4T = Eigen::Matrix<T, 4, 4>;
    using TransformT = Twist<T>;

    Vector4T res;
    Matrix4T Trans;

    TransformT Tinv = T_to_from.inverse();
    Trans = Tinv.matrix();

    res = PlaneCoeff.transpose() * Trans;
    return res;
  }

  inline Eigen::Vector3d lineIntersectPlane(const Eigen::Vector3d& lineStart,
                                            const Eigen::Vector3d& lineEnd,
                                            const Eigen::Vector4d& PlaneCoeff) {
    Eigen::Vector3d dir = lineEnd - lineStart;
    double b = lineStart.dot(PlaneCoeff.head<3>()) + PlaneCoeff.w();
    double a = dir.dot(PlaneCoeff.head<3>());

    double t = -b / a;

    Eigen::Vector3d res = lineStart + t * dir;

    return res;
  }

  template <typename T>
  bool operator()(const T* const T_w_i_host_ptr,
                  const T* const T_w_i_target_ptr,
                  const T* const T_i_c_ptr,
                  const T* const plane_d_ptr,
                  T* residuals_ptr) const {
    using Vector2T = Eigen::Matrix<T, 2, 1>;
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Vector4T = Eigen::Matrix<T, 4, 1>;
    using QuaterionT = Eigen::Quaternion<T>;
    using TransformT = Twist<T>;

    Eigen::Map<const Vector3T> Pi(T_w_i_host_ptr);
    Eigen::Map<const QuaterionT> Qi(T_w_i_host_ptr + 3);
    TransformT T_w_i_host(Qi, Pi);

    Eigen::Map<const Vector3T> Pj(T_w_i_target_ptr);
    Eigen::Map<const QuaterionT> Qj(T_w_i_target_ptr + 3);
    TransformT T_w_i_target(Qj, Pj);

    Eigen::Map<const Vector3T> tic(T_i_c_ptr);
    Eigen::Map<const QuaterionT> qic(T_i_c_ptr + 3);
    TransformT T_i_c(qic, tic);

    TransformT T_w_c_host = T_w_i_host * T_i_c;
    TransformT T_w_c_target = T_w_i_target * T_i_c;

    TransformT T_c_target_c_host = T_w_c_target.inverse() * T_w_c_host;

    T plane_d_w = plane_d_ptr[0];

    // step1: construct plane in world frame coefficient
    Vector4T plane_w;
    plane_w << T(0), T(0), T(1), plane_d_w;

    // step2: transform plane from world frame to camera frame
    Vector4T plane_c_host = TransformPlane(plane_w, T_w_c_host.inverse());

    // step3: get the 3d point in host camera frame
    Vector3T ray_c_host = pts_i_.cast<T>();
    T z_lambda =
        -plane_c_host.w() / (plane_c_host.template head<3>().dot(ray_c_host));
    Vector3T point_c_host = z_lambda * ray_c_host;

    // step4: transform the 3d point from host camera to target camera
    Vector3T point_c_target = T_c_target_c_host * point_c_host;

    Eigen::Map<Vector2T> residual(residuals_ptr);

    T point_depth_target = point_c_target.z();
    residual = (point_c_target / point_depth_target).template head<2>() -
               pts_j_.template head<2>();

    residual.applyOnTheLeft(sqrt_information_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& pts_i,
                                     const Eigen::Vector3d& pts_j) {
    return new ceres::
        AutoDiffCostFunction<ProjectionPlaneFactorAutoDiff, 2, 7, 7, 7, 1>(
            new ProjectionPlaneFactorAutoDiff(pts_i, pts_j));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Vector3d pts_i_, pts_j_;

 public:
  static Eigen::Matrix2d sqrt_information_;
};