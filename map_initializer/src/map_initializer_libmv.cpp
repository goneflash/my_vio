#include "map_initializer_libmv.hpp"

#include <iostream>

namespace vio {

MapInitializer *MapInitializer::CreateMapInitializerLIBMV() {
  MapInitializer *initializer = new MapInitializerLIBMV();
  return initializer;
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
  cv::Matx33d K_estimated;
  cv::Mat_<double> points3d_mat;
  std::vector<cv::Mat> Ps_estimated;

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

}  // vio
