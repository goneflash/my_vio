#ifndef CV2EIGEN_HELPER_
#define CV2EIGEN_HELPER_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

inline cv::Vec3d EigenVec3dToCVVec3d(const Eigen::Vector3d &p) {
  cv::Vec3d point;
  point[0] = p[0];
  point[1] = p[1];
  point[3] = p[2];
  return point;
}

#endif  // CV2EIGEN_HELPER_
