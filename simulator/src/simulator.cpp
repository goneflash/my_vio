#include "simulator.hpp"

#include <fstream>
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
        pixel[0] = -1;
        pixel[1] = -1;
      }
      /*
      std::cout << landmark.position[0] << " ";
      std::cout << landmark.position[1] << " ";
      std::cout << landmark.position[2] << " --> ";
      std::cout << pixel[0] << " ";
      std::cout << pixel[1] << std::endl;
      */
      features.push_back(pixel);
    }
    feature_pos_each_frame.push_back(features);
  }
  return true;
}

bool Simulator::WriteFeatureTracksToFile(
    std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame,
    const std::string &file_name) {
  std::ofstream output_file(file_name);
  const int num_frames = feature_pos_each_frame.size();
  const int num_features = feature_pos_each_frame[0].size();
  for (int feature_id = 0; feature_id < num_features; ++feature_id) {
    for (int frame_id = 0; frame_id < num_frames; ++frame_id) {
      // Need to check.
      // if (feature_pos_each_frame[frame_id].size() < feature_id + 1)
      //   return false;
      const Eigen::Vector2d &feature =
          feature_pos_each_frame[frame_id][feature_id];
      if (frame_id != 0) output_file << " ";
      output_file << feature[0] << " " << feature[1];
    }
    output_file << std::endl;
  }
  return true;
}
}  // namespace vio
