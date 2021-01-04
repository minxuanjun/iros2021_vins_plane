//
// Created by ubuntu on 2020/9/4.
//

#pragma once

#include <memory>
#include <opencv2/core/core.hpp>

#include "../utils/EigenTypes.h"

namespace vins {
using FeatureID = int;
using TimeFrameId = double;
using FeatureTrackerResulst = Eigen::aligned_map<
    int,
    Eigen::aligned_vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;

struct FrontEndResult {
  typedef std::shared_ptr<FrontEndResult> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // optical flow result
  double timestamp;
  FeatureTrackerResulst feature;
  cv::Mat image;
};

}  // namespace vins