#pragma once

// std
#include <iostream>
#include <map>
// Eigen
#include <eigen3/Eigen/Dense>

// ros
#include <ros/ros.h>
// self
#include "../factor/imu_factor.h"
#include "../feature_manager.h"
#include "../utils/EigenTypes.h"
#include "../utils/Twist.h"
#include "../utils/math_utils.h"

using namespace Eigen;
using namespace std;

using namespace Eigen;
using namespace std;

class ImageFrame {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImageFrame() = default;
  ImageFrame(const vins::FeatureTrackerResulst& _points, double _t)
      : timestamp{_t}, is_key_frame{false} {
    points = _points;
  };

  vins::FeatureTrackerResulst points;
  IntegrationBase* pre_integration = nullptr;
  Transformd Twi;
  double timestamp;
  bool is_key_frame = false;
};

bool VisualIMUAlignment(Eigen::aligned_map<double, ImageFrame>& all_image_frame,
                        Vector3d* Bgs,
                        Vector3d& g,
                        VectorXd& x);