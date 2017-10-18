#ifndef CV2EIGEN_HELPER_
#define CV2EIGEN_HELPER_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

inline cv::Vec3d EigenVec3dToCVVec3d(const Eigen::Vector3d &p) {
  cv::Vec3d point;
  point[0] = p[0];
  point[1] = p[1];
  point[2] = p[2];
  return point;
}

inline cv::Mat EigenVec3dToCVMat(const Eigen::Vector3d &p) {
  cv::Mat point(3, 1, CV_64F);
  point.at<double>(0) = p[0];
  point.at<double>(1) = p[1];
  point.at<double>(2) = p[2];
  return point;
}

#endif  // CV2EIGEN_HELPER_
