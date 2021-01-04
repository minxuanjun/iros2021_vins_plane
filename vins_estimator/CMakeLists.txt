cmake_minimum_required(VERSION 2.8.3)
project(vins_estimator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  graph_msgs
  eigen_stl_containers
  rviz_visual_tools
  pcl_ros
  pcl_conversions
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(OpenCV REQUIRED)
# message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
# Ceres
find_package(Ceres REQUIRED)
# Eigen
find_package(Eigen3)
# glog
find_package(Gflags)
find_package(Glog)

# PCL
find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

add_executable(vins_estimator
  src/estimator_node.cpp
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/factor/pose_local_parameterization.cpp
  src/factor/projection_td_factor.cpp
  src/factor/marginalization_factor.cpp
  src/factor/projection_factor.cpp
  src/factor/relativatepose_factor.cc
  src/factor/projection_plane_factor.cc
  #    src/factor/projection_factor_basalt.cpp
  src/utils/visualization.cpp
  src/utils/CameraPoseVisualization.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp
  src/plane_detection/plane_detection.cc
  )
  add_executable(plane_extract_test
  src/plane_extract.cpp
  )


target_link_libraries(vins_estimator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${PCL_LIBRARIES}
  )


