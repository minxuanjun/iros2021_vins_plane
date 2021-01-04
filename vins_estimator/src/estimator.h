#pragma once

// std
#include <queue>
#include <unordered_map>

// ceres
#include <ceres/ceres.h>

// opencv
#include <opencv2/core/eigen.hpp>

// ros
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

// self
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "feature_manager.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include "initial/initial_sfm.h"
#include "initial/solve_5pts.h"
#include "parameters.h"
#include "utils/math_utils.h"
#include "utils/tic_toc.h"
//#include "factor/projection_factor_basalt.h"
#include "factor/marginalization_factor.h"
#include "factor/projection_plane_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/relativepose_factor.h"
#include "utils/logging.h"

class Estimator {
 public:
  Estimator();

  void setParameter();

  // interface
  void processIMU(double t,
                  const Vector3d& linear_acceleration,
                  const Vector3d& angular_velocity);
  void processImage(const vins::FeatureTrackerResulst& image,
                    const std_msgs::Header& header);
  void setReloFrame(double _frame_stamp,
                    int _frame_index,
                    Eigen::aligned_vector<Vector3d>& _match_points,
                    Vector3d _relo_t,
                    Matrix3d _relo_r);

  // 梳理局部滑窗特征点的id
  void recomputeFrameId();

  // Estimator 重置
  void clearState();

  // visual and imu initial
  // -----------------
  bool initialStructure();
  bool visualInitialAlign();
  bool relativePose(Matrix3d& relative_R, Vector3d& relative_T, int& l);
  // -----------------
  // visual and imu initial

  void slideWindow();
  void solveOdometry();
  void slideWindowNew();
  void slideWindowOld();
  void optimization();
  void vector2double();
  void double2vector();

  inline void updateFramePose();

  bool failureDetection();

 private:
  inline int framedistance(vins::TimeFrameId frame0,
                           vins::TimeFrameId frame1) const {
    CHECK(local_active_frames.find(frame0) != local_active_frames.end() and
          local_active_frames.find(frame1) != local_active_frames.end())
        << "frame0 or frame1 out of range";

    return time_frameid2_int_frameid.at(frame0) -
           time_frameid2_int_frameid.at(frame1);
  }

 public:
  enum class SolverFlag { INITIAL, NON_LINEAR };

  enum class MarginalizationFlag { MARGIN_OLD = 0, MARGIN_SECOND_NEW = 1 };

  SolverFlag solver_flag;
  MarginalizationFlag marginalization_flag;
  Vector3d g;
  MatrixXd Ap[2], backup_A;
  VectorXd bp[2], backup_b;

  Matrix3d ric[NUM_OF_CAM];
  Vector3d tic[NUM_OF_CAM];

  Vector3d Ps[(WINDOW_SIZE + 1)];
  Vector3d Vs[(WINDOW_SIZE + 1)];
  Matrix3d Rs[(WINDOW_SIZE + 1)];
  Vector3d Bas[(WINDOW_SIZE + 1)];
  Vector3d Bgs[(WINDOW_SIZE + 1)];
  double td;

  Matrix3d back_R0, last_R, last_R0;
  Vector3d back_P0, last_P, last_P0;
  std_msgs::Header Headers[(WINDOW_SIZE + 1)];

  IntegrationBase* pre_integrations[(WINDOW_SIZE + 1)];
  Vector3d acc_0, gyr_0;

  vector<double> dt_buf[(WINDOW_SIZE + 1)];
  Eigen::aligned_vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
  Eigen::aligned_vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

  int frame_count;
  int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

  FeatureManager f_manager;
  MotionEstimator m_estimator;
  InitialEXRotation initial_ex_rotation;

  static constexpr double prior_weight = 1e8;
  bool first_imu;
  bool is_valid, is_key;
  bool failure_occur;
  bool bapply_prior = true;
  double plane_d_w_detect;

  std::array<double, 1> param_plane{};
  bool b_plane_init_success = false;

  Eigen::aligned_vector<Vector3d> point_cloud;
  Eigen::aligned_vector<Vector3d> margin_cloud;
  Eigen::aligned_vector<Vector3d> key_poses;
  double initial_timestamp;

  double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
  double para_Feature[NUM_OF_F][SIZE_POINYFEATURE];
  double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
  double para_Retrive_Pose[SIZE_POSE];
  double para_Td[1][1];
  double para_Tr[1][1];

  int loop_window_index;

  MarginalizationInfo* last_marginalization_info;
  vector<double*> last_marginalization_parameter_blocks;

  Eigen::aligned_map<double, ImageFrame> localWindowFrames;
  std::set<double> local_active_frames;
  std::map<int, double> int_frameid2_time_frameid;
  std::map<double, int> time_frameid2_int_frameid;
  IntegrationBase* tmp_pre_integration;

  // relocalization variable
  bool relocalization_info;
  double relo_frame_stamp;
  double relo_frame_index;
  int relo_frame_local_index;
  Eigen::aligned_vector<Vector3d> match_points;
  double relo_Pose[SIZE_POSE];
  Matrix3d drift_correct_r;
  Vector3d drift_correct_t;
  Vector3d prev_relo_t;
  Matrix3d prev_relo_r;
  Vector3d relo_relative_t;
  Quaterniond relo_relative_q;
  double relo_relative_yaw;

  // TODO: 增加开始optimazation的标志位
  bool b_first_marginzation_old = false;
  Transformd T_w_origin;
  // 增加视觉的voxel_map
  int64_t global_frame_cnt = 0;
  Eigen::aligned_map<vins::TimeFrameId, Transformd> local_map_poses;
};
