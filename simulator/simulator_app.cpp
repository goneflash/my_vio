#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera_model.hpp"
#include "simulator.hpp"

struct Options {
 public:
  std::string scene_file_path;
};

bool LoadCameraModel(const cv::FileNode &node, vio::Scene &scene) {
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

bool LoadLandmarks(const std::string &scene_file_name,
                             vio::Scene &scene) {

  return true;
}

bool LoadSceneFromConfigFile(const std::string &scene_file_name,
                             vio::Scene &scene) {
  cv::FileStorage scene_config;
  scene_config.open(scene_file_name, cv::FileStorage::READ);
  if (!scene_config.isOpened()) {
    std::cerr << "Error: Couldn't open scene file.\n";
    return false;
  }

  if (!LoadCameraModel(scene_config["CameraModel"], scene))
    return false;
  return true;
}

void WriteFeatureTracksToFile(
    std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame,
    const std::string &file_name) {
}

void PrintUsage();

int main(int argc, char **argv) {
  Options option;
  for (int i = 0; i < argc; ++i) {
    if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--scene_file_path")) {
      option.scene_file_path = argv[++i];
    }
  }
  if (option.scene_file_path.empty()) {
    PrintUsage();
    return -1;
  }

  vio::Scene scene;
  if (!LoadSceneFromConfigFile(option.scene_file_path, scene))
    return false;

  vio::Simulator simulator;
  std::vector<std::vector<Eigen::Vector2d>> feature_pos_each_frame;
  if (!simulator.GenerateFeatureTracksFromTranslationTrajectory(
        scene, feature_pos_each_frame))
    return false;

  return 0;
}

void PrintUsage() {
  std::cout << "Error. Unknown arguments.\n";
  std::cout << "Usage: \n";
  std::cout << "       simulator_app\n";
  std::cout << "            -p, --scene_file_path file path of scene to be simulated.\n";
}
