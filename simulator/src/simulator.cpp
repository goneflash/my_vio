#include "simulator.hpp"

#include <iostream>

namespace vio {

bool Simulator::GenerateFeatureTracksFromTranslationTrajectory(
    const Scene &scene,
    std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame) {
  feature_pos_each_frame.clear();
  for (const auto camera_center : scene.trajectory) {
    std::vector<Eigen::Vector2d> features;
    for (const auto landmark : scene.landmarks) {
      Eigen::Vector2d pixel;
      if (!scene.camera->ProjectPointToPixel(landmark, pixel)) {
        std::cout << "error";
        continue;
        //return false;
      }
    
      std::cout << landmark[0] << " ";
      std::cout << landmark[1] << " ";
      std::cout << landmark[2] << " --> ";
      std::cout << pixel[0] << " ";
      std::cout << pixel[1] << std::endl;

      features.push_back(pixel);
    }
    feature_pos_each_frame.push_back(features);
  }

  return true;
}


} // namespace vio
