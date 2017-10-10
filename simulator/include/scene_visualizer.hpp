#ifndef SCENE_VISUALIZER_
#define SCENE_VISUALIZER_

#ifdef OPENCV_VIZ_FOUND

#include <string>

#include <opencv2/viz.hpp>

#include "scene.hpp"

namespace vio {

class SceneVisualizer {
 public:
  explicit SceneVisualizer(const std::string &name) {
    window_ = std::unique_ptr<cv::viz::Viz3d>(new cv::viz::Viz3d(name));
    window_->setWindowSize(cv::Size(500, 500));
    window_->setWindowPosition(cv::Point(150, 150));
    window_->setBackgroundColor();  // black by default
  }

  SceneVisualizer() = delete;

  void VisualizeScene(const Scene &scene);
  void Render();

  void SetCameraPoses(const std::vector<CameraPose> &poses);
  void AddCameraPose(const CameraPose &pose);

  void SetLandmarks(const std::vector<Landmark> &landmarks);
  void SetLandmarks(const std::vector<cv::Point3f> &landmarks);
  void AddLandmarks(const std::vector<Landmark> &landmarks);

 private:
  void RenderCameraPoses();
  void RenderPoints();

  std::unique_ptr<cv::viz::Viz3d> window_;
  std::vector<cv::Affine3d> path_;
  std::vector<cv::Vec3d> point_cloud_;
};

}  // vio

#endif  // OPENCV_VIZ_FOUND

#endif  // SCENE_VISUALIZER_
