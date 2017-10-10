#ifdef OPENCV_VIZ_FOUND

#include "scene.hpp"
#include "scene_visualizer.hpp"
// TODO: Should be util/cv2eigen_helper.hpp
#include "cv2eigen_helper.hpp"

namespace vio {

void SceneVisualizer::VisualizeScene(const Scene &scene) {
  SetCameraPoses(scene.trajectory);
  SetLandmarks(scene.landmarks);
  Render();
}

void SceneVisualizer::SetCameraPoses(const std::vector<CameraPose> &poses) {
  path_.clear();
  for (size_t i = 0; i < poses.size(); ++i)
    path_.push_back(cv::Affine3d(poses[i].R, poses[i].t));
}
void SceneVisualizer::AddCameraPose(const CameraPose &pose) {
  path_.push_back(cv::Affine3d(pose.R, pose.t));
}

void SceneVisualizer::SetLandmarks(const std::vector<cv::Point3f> &landmarks) {
  for (size_t i = 0; i < landmarks.size(); ++i) {
    point_cloud_.push_back(
        cv::Vec3d(landmarks[i].x, landmarks[i].y, landmarks[i].z));
  }
}

void SceneVisualizer::SetLandmarks(const std::vector<Landmark> &landmarks) {
  for (size_t i = 0; i < landmarks.size(); ++i) {
    point_cloud_.push_back(EigenVec3dToCVVec3d(landmarks[i].position));
  }
}

void SceneVisualizer::AddLandmarks(const std::vector<Landmark> &landmarks) {}

void SceneVisualizer::Render() {
  RenderCameraPoses();
  RenderPoints();

  window_->spin();
}

void SceneVisualizer::RenderCameraPoses() {
  if (path_.size() > 0) {
    window_->showWidget("cameras_frames_and_lines",
                        cv::viz::WTrajectory(path_, cv::viz::WTrajectory::BOTH,
                                             0.1, cv::viz::Color::green()));
    // TODO: For now, it doesn't matter the K.
    window_->showWidget("cameras_frustums", cv::viz::WTrajectoryFrustums(
                                                path_, cv::Vec2d(0.78, 0.78),
                                                0.1, cv::viz::Color::yellow()));
    window_->setViewerPose(path_[0]);
  } else {
    std::cout << "Cannot render the cameras: Empty path" << std::endl;
  }
}

void SceneVisualizer::RenderPoints() {
  if (!point_cloud_.size()) {
    std::cout << "Cannot render points: Empty pointcloud" << std::endl;
    return;
  }
  cv::viz::Color point_color = cv::viz::Color::green();
  cv::viz::WCloud cloud_widget(point_cloud_, point_color);
  std::string cloud_name = "point_cloud";
  window_->showWidget(cloud_name, cloud_widget);
}

}  // vio

#endif
