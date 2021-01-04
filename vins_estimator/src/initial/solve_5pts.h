#pragma once
// std
#include <vector>

// opencv
#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
// Eigen
#include <Eigen/Dense>

// ros
#include <ros/console.h>

// self
#include "../utils/EigenTypes.h"

using namespace std;
using namespace Eigen;
class MotionEstimator {
 public:
  bool solveRelativeRT(const Eigen::aligned_vector<
                           pair<Eigen::Vector3d, Eigen::Vector3d>>& corres,
                       Eigen::Matrix3d& R,
                       Eigen::Vector3d& T);

 private:
  double testTriangulation(const vector<cv::Point2f>& l,
                           const vector<cv::Point2f>& r,
                           cv::Mat_<double> R,
                           cv::Mat_<double> t);
  void decomposeE(cv::Mat E,
                  cv::Mat_<double>& R1,
                  cv::Mat_<double>& R2,
                  cv::Mat_<double>& t1,
                  cv::Mat_<double>& t2);
};
