#include "pnp_estimator_ocv.hpp"

#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>

namespace vio {

std::unique_ptr<PnPEstimator> PnPEstimator::CreatePnPEstimatorOCV() {
  return std::unique_ptr<PnPEstimator>(new PnPEstimatorOCV());
}

bool PnPEstimatorOCV::EstimatePose(const std::vector<cv::Point2f> &image_points,
                                   const std::vector<cv::Point3f> &points3d,
                                   const cv::Mat &K, std::vector<bool> &inliers,
                                   cv::Mat &R_est, cv::Mat &t_est) {
  int iter_count = 500;
  float reprojection_error = 1.0;
  double confidence = 0.999;

  cv::Mat distCoeffs =
      cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
  cv::Mat R_est_vec = cv::Mat::zeros(3, 1, CV_64FC1);  // output rotation vector
  t_est = cv::Mat::zeros(3, 1, CV_64FC1);  // output translation vector

  // if true the function uses the provided rvec and tvec values as
  // initial approximations of the rotation and translation vectors
  bool useExtrinsicGuess = false;

  cv::Mat inliers_mat;
  int pnp_method = CV_ITERATIVE;

  cv::solvePnPRansac(points3d, image_points, K, distCoeffs, R_est_vec, t_est,
                     useExtrinsicGuess, iter_count, reprojection_error,
                     confidence, inliers_mat, pnp_method);
  std::cout << "PnP found " << inliers_mat.rows << " / " << points3d.size()
            << " match inliers.\n";

  inliers.resize(points3d.size(), false);
  for (int i = 0; i < inliers_mat.rows; ++i) {
    const int inlier_id = inliers_mat.at<int>(i);
    inliers[inlier_id] = true;
  }

  cv::Rodrigues(R_est_vec, R_est);

  return true;
}

}  // vio
