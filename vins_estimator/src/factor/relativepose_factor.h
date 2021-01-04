//
// Created by ubuntu on 2020/12/31.
//

#pragma once

// Eigen
#include "../utils/EigenTypes.h"
// Ceres
#include <ceres/ceres.h>
// ros
#include <ros/assert.h>

// self
#include "../parameters.h"
#include "../utils/math_utils.h"
#include "../utils/tic_toc.h"

class RelativePoseFactorAutoDiff {
 public:
  RelativePoseFactorAutoDiff(const Eigen::Vector3d& t_meas,
                             const Eigen::Quaterniond& q_meas) {
    t_meas_ = t_meas;
    q_meas_ = q_meas;
  };

  template <typename T>
  bool operator()(const T* const pose_i, T* residuals_ptr) const {
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> Pi(pose_i);
    Eigen::Map<const Eigen::Quaternion<T>> Qi(pose_i + 3);

    Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals_ptr);

    residual.template head<3>() = Pi - t_meas_.cast<T>();
    residual.template tail<3>() =
        T(2) * (Qi.conjugate() * q_meas_.cast<T>()).vec();
    residual.applyOnTheLeft(sqrt_info_.template cast<T>());

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d& t_meas,
                                     const Eigen::Quaterniond& q_meas) {
    return new ceres::AutoDiffCostFunction<RelativePoseFactorAutoDiff, 6, 7>(
        new RelativePoseFactorAutoDiff(t_meas, q_meas));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  Eigen::Vector3d t_meas_;
  Eigen::Quaterniond q_meas_;

  static Eigen::Mat66d sqrt_info_;
  static double sum_t;
};