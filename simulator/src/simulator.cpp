#include "simulator.hpp"

#include <iostream>

#include "cv2eigen_helper.hpp"

namespace vio {

bool Simulator::GenerateFeatureMeasurementsFromTrajectory(
    const Scene &scene,
    std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame) {
  feature_pos_each_frame.clear();
  for (const auto camera_view : scene.trajectory) {
    std::vector<Eigen::Vector2d> features;
    for (const auto landmark : scene.landmarks) {
      // Transform
      cv::Mat point = cv::Mat(3, 1, CV_64F);
      point.at<double>(0) = landmark.position[0];
      point.at<double>(1) = landmark.position[1];
      point.at<double>(2) = landmark.position[2];
      const cv::Mat transformed_point =
          camera_view.R * point + cv::Mat(camera_view.t);

      Eigen::Vector3d p;
      p[0] = transformed_point.at<double>(0);
      p[1] = transformed_point.at<double>(1);
      p[2] = transformed_point.at<double>(2);

      Eigen::Vector2d pixel;
      if (!scene.camera->ProjectPointToPixel(p, pixel)) {
        std::cout << "Warning: point not in camera view.\n";
        continue;
        // return false;
      }

      std::cout << landmark.position[0] << " ";
      std::cout << landmark.position[1] << " ";
      std::cout << landmark.position[2] << " --> ";
      std::cout << pixel[0] << " ";
      std::cout << pixel[1] << std::endl;

      features.push_back(pixel);
    }
    feature_pos_each_frame.push_back(features);
  }

  return true;
}

}  // namespace vio
