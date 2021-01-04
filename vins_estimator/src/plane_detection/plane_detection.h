//
// Created by ubuntu on 2020/12/31.
//

#pragma once

// std
#include <algorithm>
#include <vector>
// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// ros
#include <sensor_msgs/PointCloud.h>

#include "nanoflann.hpp"
// 需要

struct PointCloudAdaptor {
  const std::vector<geometry_msgs::Point32>& points_;

  const size_t num_points;

  PointCloudAdaptor(const std::vector<geometry_msgs::Point32>& points)
      : points_(points), num_points(points.size()) {}

  inline size_t kdtree_get_point_count() const { return num_points; }

  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0) return points_[idx].x;
    if (dim == 1) return points_[idx].y;
    if (dim == 2) return points_[idx].z;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

class plane_detection {
 public:
  plane_detection() {
    index_max_ = static_cast<int>((dmax_ - dmin_) / dresoluation_);
    acculators_.resize(index_max_);
    std::fill(acculators_.begin(), acculators_.end(), 0);
  }

  ~plane_detection() = default;

  int dist2index(double dist) const {
    int index = static_cast<int>((dist - dmin_) / dresoluation_ + 0.5);
    return index;
  }

  double index2dist(int index) const {
    double dist = (index - 0.5) * dresoluation_ + dmin_;
    return dist;
  }

  Eigen::Vector3d convertPoint32(const geometry_msgs::Point32& point) {
    Eigen::Vector3d point_eigen;
    point_eigen[0] = point.x;
    point_eigen[1] = point.y;
    point_eigen[2] = point.z;
    return point_eigen;
  }

  geometry_msgs::Point32 convertPoint32(const Eigen::Vector3d& point) {
    geometry_msgs::Point32 point32_msg;
    point32_msg.x = point[0];
    point32_msg.y = point[1];
    point32_msg.z = point[2];
    return point32_msg;
  }

  double detect_plane(const sensor_msgs::PointCloud& pointCloud,
                      const Eigen::Vector3d& camera_center);

 public:
  // 平面系数d

  std::vector<double> acculators_;
  double dresoluation_ = 0.1;
  double dmax_ = 2.3;
  double dmin_ = -2.3;
  int index_max_;
  double d_;
};
