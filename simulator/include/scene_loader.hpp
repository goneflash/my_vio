#ifndef SCENE_LOADER_
#define SCENE_LOADER_

#include <opencv2/opencv.hpp>

#include "camera_model.hpp"
#include "scene.hpp"

namespace vio {

class SceneLoader {
 public:
  SceneLoader() {}

  bool LoadSceneFromConfigFile(const std::string &scene_file_name,
                               vio::Scene &scene);
 private:
  bool LoadCameraModel(const cv::FileNode &node, vio::Scene &scene);
  bool LoadLandmarks(const cv::FileNode &node, vio::Scene &scene);
  bool LoadCameraPoses(const cv::FileNode &node, vio::Scene &scene);

};
} // vio
#endif // SCENE_LOADER_
