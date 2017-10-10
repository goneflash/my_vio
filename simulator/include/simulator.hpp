#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include <memory>
#include <string>
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
  bool GenerateFeatureMeasurementsFromTrajectory(
      const Scene &scene,
      std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame);

  bool WriteFeatureTracksToFile(
      std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame,
      const std::string &file_name);
};

}  // namespace vio

#endif
