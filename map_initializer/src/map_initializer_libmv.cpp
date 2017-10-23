#ifdef SFM_FOUND

#include "map_initializer_libmv.hpp"

#include <iostream>

namespace vio {

std::unique_ptr<MapInitializer> MapInitializer::CreateMapInitializerLIBMV() {
  std::unique_ptr<MapInitializer> initializer =
      std::unique_ptr<MapInitializer>(new MapInitializerLIBMV());
  return std::move(initializer);
}

bool MapInitializerLIBMV::Initialize(
    const std::vector<std::vector<cv::Vec2d> > &feature_vectors,
    const cv::Mat &K, std::vector<cv::Point3f> &points3d,
    std::vector<bool> &points3d_mask, std::vector<cv::Mat> &Rs,
    std::vector<cv::Mat> &ts) {
  if (feature_vectors.size() < 2) {
    std::cerr << "Error: libmv initializer not support views < 3.\n";
    return false;
  }

  if (feature_vectors.size() == 2) {
    std::cout << "Only two frames given. Initialize from two views.\n";
    return InitializeTwoFrames(feature_vectors[0], feature_vectors[1], K,
                               points3d, Rs, ts);
  }

  std::vector<cv::Mat> all_2d_points;
  const int num_frame = feature_vectors.size();
  const int num_features = feature_vectors[0].size();
  for (int i = 0; i < num_frame; ++i) {
    if (num_features != feature_vectors[i].size()) return false;

    cv::Mat_<double> frame(2, num_features);
    for (int j = 0; j < num_features; ++j) {
      frame(0, j) = feature_vectors[i][j][0];
      frame(1, j) = feature_vectors[i][j][1];
    }
    all_2d_points.push_back(cv::Mat(frame));
  }

  cv::Mat refined_camera_matrix = cv::Mat(K).clone();
  std::vector<cv::Mat> points3d_mat;
  cv::sfm::reconstruct(all_2d_points, Rs, ts, refined_camera_matrix,
                       points3d_mat, true);

  // Convert mat to point3f
  points3d.clear();
  for (int i = 0; i < points3d_mat.size(); ++i) {
    points3d.push_back(cv::Point3f(points3d_mat.at(i)));
  }

  std::cout << "\n--------Initialization--------------------\n" << std::endl;
  std::cout << "2D feature number: " << feature_vectors[0].size() << std::endl;
  std::cout << "Initialized 3D points: " << points3d.size() << std::endl;
  std::cout << "Estimated cameras: " << Rs.size() << std::endl;
  std::cout << "Original intrinsics: " << std::endl
            << K << std::endl;
  std::cout << "Refined intrinsics: " << std::endl
            << refined_camera_matrix << std::endl
            << std::endl;
  std::cout << "Cameras are: " << std::endl;
  for (int i = 0; i < Rs.size(); ++i) {
    std::cout << "R: " << std::endl
              << Rs[i] << std::endl;
    std::cout << "t: " << std::endl
              << ts[i] << std::endl;
  }
  std::cout << "\n----------------------------\n" << std::endl;
  return true;
}

int EvaluateSolutionRT(const cv::Mat &R, const cv::Mat &t, const cv::Mat &K,
                       const std::vector<cv::Vec2d> &kp0,
                       const std::vector<cv::Vec2d> &kp1,
                       const std::vector<bool> &match_inliers,
                       std::vector<cv::Point3f> &points_3d,
                       std::vector<bool> &points3d_mask);

bool MapInitializerLIBMV::InitializeTwoFrames(
    const std::vector<cv::Vec2d> &kp0, const std::vector<cv::Vec2d> &kp1,
    const cv::Mat &K, std::vector<cv::Point3f> &points3d,
    std::vector<cv::Mat> &Rs, std::vector<cv::Mat> &ts) {
  if (kp0.size() != kp1.size()) {
    std::cerr << "Error: keypoints number of two frames not match. Quit.\n";
    return false;
  }
  // Convert keypoints to cv::sfm format.
  int num_pts = kp0.size();
  cv::Mat_<double> x1 = cv::Mat_<double>(2, num_pts);
  cv::Mat_<double> x2 = cv::Mat_<double>(2, num_pts);

  for (int i = 0; i < num_pts; ++i) {
    x1(0, i) = kp0[i][0];
    x1(1, i) = kp0[i][1];
    x2(0, i) = kp1[i][0];
    x2(1, i) = kp1[i][1];
  }

  std::vector<cv::Mat_<double> > points2d;
  points2d.push_back(x1);
  points2d.push_back(x2);
  cv::Mat K_estimated(K);
  cv::Mat_<double> points3d_mat;
  std::vector<cv::Mat> Ps_estimated;

  // Output R, t to evaluate solution.
  {
    std::vector<cv::Mat> Rs_estimated;
    std::vector<cv::Mat> ts_estimated;
    cv::sfm::reconstruct(points2d, Rs_estimated, ts_estimated, K_estimated,
                         points3d_mat, true);
    std::cout << "Reconstruct to R and t.\n";
    std::cout << "Number of cameras: " << Rs_estimated.size() << std::endl;
    for (int i = 0; i < Rs_estimated.size(); ++i) {
      std::cout << "Camera " << i << " :"
                << "R:\n";
      std::cout << Rs_estimated[i] << std::endl;
      std::cout << "t:\n" << ts_estimated[i] << std::endl;
      std::vector<cv::Point3f> tmp_points3d;
      std::vector<bool> triangulated_mask;  // not used yet
      std::vector<bool> match_inliers(kp0.size(), true);
      int num_point_inlier =
          EvaluateSolutionRT(Rs[i], ts[i], K, kp0, kp1, match_inliers,
                             tmp_points3d, triangulated_mask);
    }
  }

  cv::sfm::reconstruct(points2d, Ps_estimated, points3d_mat, K_estimated, true);
  // Convert mat to point3f
  points3d.clear();
  for (int i = 0; i < num_pts; ++i) {
    cv::Point3f point3d((float)points3d_mat(0, i), (float)points3d_mat(1, i),
                        (float)points3d_mat(2, i));
    points3d.push_back(point3d);
  }
  std::cout << "\n--------Initialization--------------------\n" << std::endl;
  std::cout << "2D feature number: " << kp0.size() << std::endl;
  std::cout << "Initialized 3D points: " << points3d.size() << std::endl;
  std::cout << "Original intrinsics: " << std::endl
            << K << std::endl;
  std::cout << "Refined intrinsics: " << std::endl
            << K_estimated << std::endl
            << std::endl;
  std::cout << "Cameras are: " << std::endl;
  for (int i = 0; i < Ps_estimated.size(); ++i) {
    std::cout << "P: " << std::endl
              << Ps_estimated[i] << std::endl;
  }
  std::cout << "\n----------------------------\n" << std::endl;
  return true;
}

int EvaluateSolutionRT(const cv::Mat &R, const cv::Mat &t, const cv::Mat &K,
                       const std::vector<cv::Vec2d> &kp0,
                       const std::vector<cv::Vec2d> &kp1,
                       const std::vector<bool> &match_inliers,
                       std::vector<cv::Point3f> &points_3d,
                       std::vector<bool> &points3d_mask) {
  // Calibration parameters
  const double fx = K.at<double>(0, 0);
  const double fy = K.at<double>(1, 1);
  const double cx = K.at<double>(0, 2);
  const double cy = K.at<double>(1, 2);

  // points_3d.resize(kp0.size());

  // std::vector<double> vCosParallax;
  // vCosParallax.reserve(kp0.size());

  // Camera 1 Projection Matrix K[I|0]
  cv::Mat P0(3, 4, CV_64F, cv::Scalar(0));
  K.copyTo(P0.rowRange(0, 3).colRange(0, 3));

  cv::Mat O1 = cv::Mat::zeros(3, 1, CV_64F);

  // Camera 2 Projection Matrix K[R|t]
  cv::Mat P1(3, 4, CV_64F);
  R.copyTo(P1.rowRange(0, 3).colRange(0, 3));
  t.copyTo(P1.rowRange(0, 3).col(3));
  P1 = K * P1;

  cv::Mat O2 = -R.t() * t;

  int nGood = 0;
  int nInfinite = 0;
  int nParallal = 0;
  int nLargeError = 0;
  int nNegativeDepth = 0;
  for (int i = 0; i < match_inliers.size(); ++i) {
    if (!match_inliers[i]) continue;

    cv::Mat p3dC1(3, 1, CV_64F);
    cv::Point3f point3d;
    TriangulateDLT(kp0[i], kp1[i], P0, P1, point3d);

    p3dC1.at<double>(0) = point3d.x;
    p3dC1.at<double>(1) = point3d.y;
    p3dC1.at<double>(2) = point3d.z;

    cv::Mat p3dC2 = R * p3dC1 + t;
    float depth1 = point3d.z;
    float depth2 = p3dC2.at<double>(2);

    if (depth1 <= 0 || depth2 <= 0) {
      points3d_mask.push_back(false);
      points_3d.push_back(cv::Point3f(0, 0, 0));
      nNegativeDepth++;
      continue;
    }

    // TODO: Make sure isfinite is in std
    if (!std::isfinite(p3dC1.at<double>(0)) ||
        !std::isfinite(p3dC1.at<double>(1)) ||
        !std::isfinite(p3dC1.at<double>(2))) {
      nInfinite++;
      points3d_mask.push_back(false);
      points_3d.push_back(cv::Point3f(0, 0, 0));
      continue;
    }

    // Check parallax
    cv::Mat normal1 = p3dC1 - O1;
    double dist1 = cv::norm(normal1);
    cv::Mat normal2 = p3dC1 - O2;
    double dist2 = cv::norm(normal2);
    double cosParallax = normal1.dot(normal2) / (dist1 * dist2);

    if (cosParallax > 0.9998) {
      nParallal++;
      points3d_mask.push_back(false);
      points_3d.push_back(cv::Point3f(0, 0, 0));
      continue;
    }

    double error0 = ComputeReprojectionError(point3d, kp0[i], P0);
    double error1 = ComputeReprojectionError(point3d, kp1[i], P1);

    if (error0 > 5 || error1 > 5) {
      nLargeError++;
      points3d_mask.push_back(false);
      points_3d.push_back(cv::Point3f(0, 0, 0));
      continue;
    }

    nGood++;
    points3d_mask.push_back(true);
    points_3d.push_back(point3d);
    /*
           // std::cout << "Parallax: " << cosParallax << std::endl;
           // Check depth in front of first camera (only if enough parallax, as
           // "infinite" points can easily go to negative depth)
           if (p3dC1.at<double>(2) <= 0 && cosParallax < 0.99998) continue;

           // Check depth in front of second camera (only if enough parallax, as
           // "infinite" points can easily go to negative depth)
           cv::Mat p3dC2 = R * p3dC1 + t;

           if (p3dC2.at<double>(2) <= 0 && cosParallax < 0.99998) continue;

           // Check reprojection error in first image
           double im1x, im1y;
           double invZ1 = 1.0 / p3dC1.at<double>(2);
           im1x = fx * p3dC1.at<double>(0) * invZ1 + cx;
           im1y = fy * p3dC1.at<double>(1) * invZ1 + cy;

           double squareError1 = (im1x - kp0[i][0]) * (im1x - kp0[i][0]) +
                                 (im1y - kp0[i][1]) * (im1y - kp0[i][1]);

           if (squareError1 > th2) continue;

           // Check reprojection error in second image
           double im2x, im2y;
           double invZ2 = 1.0 / p3dC2.at<double>(2);
           im2x = fx * p3dC2.at<double>(0) * invZ2 + cx;
           im2y = fy * p3dC2.at<double>(1) * invZ2 + cy;

           double squareError2 = (im2x - kp1[i][0]) * (im2x - kp1[i][0]) +
                                 (im2y - kp1[i][1]) * (im2y - kp1[i][1]);

           if (squareError2 > th2) continue;

           vCosParallax.push_back(cosParallax);
           points_3d[i] = cv::Point3f(p3dC1.at<double>(0), p3dC1.at<double>(1),
                                      p3dC1.at<double>(2));
           nGood++;

           if (cosParallax < 0.99998) vbGood[i] = true;
       */
  }
  /*
    if (nGood > 0) {
      sort(vCosParallax.begin(), vCosParallax.end());

      size_t idx = std::min(50, int(vCosParallax.size() - 1));
      parallax = acos(vCosParallax[idx]) * 180 / CV_PI;
    } else
      parallax = 0;
  */
  std::cout << "Found " << nInfinite << " / " << kp0.size()
            << " infinite points during triangulation.\n";
  std::cout << "Found " << nParallal << " / " << kp0.size()
            << " parallal points during triangulation.\n";
  std::cout << "Found " << nLargeError << " / " << kp0.size()
            << " large error points during triangulation.\n";
  std::cout << "Found " << nNegativeDepth << " / " << kp0.size()
            << " negative depth points during triangulation.\n";

  return nGood;
}
}  // vio

#endif
