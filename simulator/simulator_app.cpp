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
  const std::string camera_type = (std::string)node["Camera Type"];
  if (camera_type == "Pinhole") {
    vio::PinholeCameraModel<double>::ParamsArray params;
    std::unique_ptr<vio::CameraModel<double>> camera =
        std::unique_ptr<vio::CameraModel<double>>(
            new vio::PinholeCameraModel<double>(480, 640, params));
    scene.SetCameraModel(std::move(camera));
  } else {
    return false;
  }
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

  return 0;
}

void PrintUsage() {
  std::cout << "Error. Unknown arguments.\n";
  std::cout << "Usage: \n";
  std::cout << "       simulator_app\n";
  std::cout << "            -p, --scene_file_path file path of scene to be simulated.\n";
}
