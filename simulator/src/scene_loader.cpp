
#include "scene.hpp"
#include "scene_loader.hpp"

namespace vio{

bool SceneLoader::LoadSceneFromConfigFile(const std::string &scene_file_name,
                             vio::Scene &scene) {
  cv::FileStorage scene_config;
  scene_config.open(scene_file_name, cv::FileStorage::READ);
  if (!scene_config.isOpened()) {
    std::cerr << "Error: Couldn't open scene file.\n";
    return false;
  }

  if (!LoadCameraModel(scene_config["CameraModel"], scene))
    return false;
  if (!LoadLandmarks(scene_config["Landmarks"], scene))
    return false;
  if (!LoadCameraPoses(scene_config["CameraPoses"], scene))
    return false;
  return true;
}

bool SceneLoader::LoadCameraModel(const cv::FileNode &node, vio::Scene &scene) {
  const std::string camera_type = (std::string)node["CameraType"];
  if (camera_type == "Pinhole") {
    // Load parameters.
    int num_params = (int)node["NumParams"];
    cv::Mat params_cv;
    node["Params"] >> params_cv;
    vio::PinholeCameraModel<double>::ParamsArray params;
    // TODO: Check size of params == num_params.
    for (int i = 0; i < num_params; ++i) {
      params[i] = params_cv.at<double>(i);
      std::cout << "Param " << i << " : " << params[i] << std::endl;
    }

    std::unique_ptr<vio::CameraModel<double>> camera =
        std::unique_ptr<vio::CameraModel<double>>(
            new vio::PinholeCameraModel<double>(480, 640, params));
    scene.SetCameraModel(std::move(camera));
  } else {
    std::cerr << "Error: Unknown camera model.\n";
    return false;
  }
  return true;
}

bool SceneLoader::LoadLandmarks(const cv::FileNode &node, vio::Scene &scene) {
  int num_landmarks = (int)node["NumLandmarks"];
  scene.landmarks.resize(num_landmarks);
  cv::Mat landmarks_cv;
  node["Positions"] >> landmarks_cv;
  for (int i = 0; i < num_landmarks; ++i) {
    scene.landmarks[i].position[0] = landmarks_cv.at<double>(i, 0);
    scene.landmarks[i].position[1] = landmarks_cv.at<double>(i, 1);
    scene.landmarks[i].position[2] = landmarks_cv.at<double>(i, 2);
  }

  return true;
}

bool SceneLoader::LoadCameraPoses(const cv::FileNode &node, vio::Scene &scene) {
  int num_poses = (int)node["NumPoses"];
  scene.trajectory.resize(num_poses);
  cv::Mat poses_cv;
  node["Poses"] >> poses_cv;
  for (int i = 0; i < num_poses; ++i) {
    vio::CameraPose new_pose;
    new_pose.timestamp = i;
    new_pose.position[0] = poses_cv.at<double>(i, 0);
    new_pose.position[1] = poses_cv.at<double>(i, 1);
    new_pose.position[2] = poses_cv.at<double>(i, 2);
  }

  return true;
}
} // vio
