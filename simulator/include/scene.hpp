#ifndef SIMULATOR_SCENE_HPP_
#define SIMULATOR_SCENE_HPP_

#include <memory>

#include "camera_model.hpp"

namespace vio {

struct CameraPose {
  CameraPose() : timestamp(0) {
    position << 0, 0, 0;
    orientation << 0, 0, 0, 1;
  }
  double timestamp;
  Eigen::Vector3d position;
  Eigen::Vector4d orientation;
};

struct Landmark {
  Eigen::Vector3d position;
};

/*
 * Contains 
 * 1) Camera Intrisics
 * 2) 3D points as features
 * 2) Camera trajectory
 */
class Scene {
 public:
  bool SetCameraModel(std::unique_ptr<CameraModel<double>> model) {
    if (!model)
      return false;
    camera = std::move(model);
    return true;
  }

  std::unique_ptr<CameraModel<double>> camera;
  std::vector<CameraPose> trajectory;
  std::vector<Landmark> landmarks;
};

} // namespace vio

#endif
