#ifndef SIMULATOR_SCENE_HPP_
#define SIMULATOR_SCENE_HPP_

#include <memory>

#include <opencv2/opencv.hpp>

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

  // TODO: Remove this. Only use eigen.
  cv::Mat R;
  cv::Vec3d t;
};

struct Landmark {
  Eigen::Vector3d position;
};

/*
 * Contains
 * 1) Camera Intrisics
 *    * Default is Pinhole model.
 * 2) 3D points as features
 * 2) Camera trajectory
 */
class Scene {
 public:
  Scene() {
    vio::PinholeCameraModel<double>::ParamsArray params;
    params << 650, 650, 320, 240;
    camera = std::unique_ptr<vio::CameraModel>(
        new vio::PinholeCameraModel<double>(480, 640, params));
  }

  bool SetCameraModel(std::unique_ptr<CameraModel> model) {
    if (!model) return false;
    camera = std::move(model);
    return true;
  }

  std::unique_ptr<CameraModel> camera;
  std::vector<CameraPose> trajectory;
  std::vector<Landmark> landmarks;
};

}  // namespace vio

#endif
