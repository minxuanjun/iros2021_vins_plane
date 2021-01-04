//
// Created by ubuntu on 2020/12/31.
//

#include "plane_detection.h"

// glog
#include <glog/logging.h>

double plane_detection::detect_plane(const sensor_msgs::PointCloud& pointCloud,
                                     const Eigen::Vector3d& camera_center) {
  std::fill(acculators_.begin(), acculators_.end(), 0);

  const PointCloudAdaptor pc2kd(pointCloud.points);
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
      PointCloudAdaptor,
      3>;
  KDTree index(
      3,
      pc2kd,
      nanoflann::KDTreeSingleIndexAdaptorParams(
          10));  // arguments: dimensionality, point cloud adaptor, max leafs
  index.buildIndex();

  const float search_radius_sq = 2.5 * dmax_ * 2.5 * dmax_;
  std::vector<std::pair<size_t, float>> ret_matches;
  nanoflann::SearchParams params;
  params.sorted = false;  // do not sort matches by distance to query point

  Eigen::Vector3f camera_center_f32 = camera_center.cast<float>();

  size_t num_neighbors = index.radiusSearch(
      camera_center_f32.data(), search_radius_sq, ret_matches, params);

  Eigen::Vector3d z_axis{0, 0, -1};
  double camera_z = camera_center.z();
  for (size_t sec = 0; sec < num_neighbors; ++sec) {  // pair points
    Eigen::Vector3d pi =
        convertPoint32(pointCloud.points[ret_matches[sec].first]);

    double cannidate_d = pi.z() - camera_z;

    if (cannidate_d > dmin_ && cannidate_d < dmax_) {
      int index = dist2index(cannidate_d);
      if (index >= acculators_.size()) index = acculators_.size() - 1;
      if (index < 0) index = 0;
      ++acculators_.at(index);  // actual voting
    }
  }

  auto d_best_candidate_ptr =
      std::max_element(acculators_.begin(), acculators_.end());
  auto d_best_candidate =
      index2dist(d_best_candidate_ptr - acculators_.begin());

  LOG(INFO) << "\ncadidate index: "
            << d_best_candidate_ptr - acculators_.begin()
            << "\ndisttance: " << d_best_candidate
            << "\nthe number of point: " << *d_best_candidate_ptr;
  return d_best_candidate;
}
