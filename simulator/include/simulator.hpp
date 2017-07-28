#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <vector>

#include <Eigen/Dense>

#include "camera_model.hpp"

namespace vio {
/*
 * Input camera parameters, camera trajectory and 3D points.
 * Output feature position of each frame.
 *
 */
class Simulator {
 public:
   /*
    * Trajectory is a vector of position of camera centers.
    * Camera direction is always facing axis z, in right hand coordinate system.
    */
   // TODO: Add camera model factory.
   template<class CameraModel>
   bool GenerateFeatureTracksFromTranslationTrajectory(
       const CameraModel &camera, const std::vector<Eigen::Vector3d> &trajectory,
       const std::vector<Eigen::Vector3d> &points_3d,
       std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame);
};

template<class CameraModel>
bool Simulator::GenerateFeatureTracksFromTranslationTrajectory(
  const CameraModel &camera, const std::vector<Eigen::Vector3d> &trajectory,
  const std::vector<Eigen::Vector3d> &points_3d,
  std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame) {
  feature_pos_each_frame.clear();
  for (auto camera_center : trajectory) {
    std::vector<Eigen::Vector2d> features;
    for (auto point_3d : points_3d) {
      Eigen::Vector2d pixel;
      if (!camera.ProjectPoint(point_3d, pixel))
        return false;
      features.push_back(pixel);
    }
    feature_pos_each_frame.push_back(features);
  }

  return true;
}

} // namespace vio

#endif
