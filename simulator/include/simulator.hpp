#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "camera_model.hpp"
#include "scene.hpp"

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
  bool GenerateFeatureTracksFromTranslationTrajectory(
      const Scene &scene,
      std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame);
};

}  // namespace vio

#endif
