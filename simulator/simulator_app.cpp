#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera_model.hpp"
#include "simulator.hpp"
#include "scene_generator.hpp"
#include "scene_loader.hpp"
#include "scene_visualizer.hpp"
#include "scene_exporter.hpp"

struct Options {
 public:
  std::string scene_file_path;
};

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

  /*
  vio::Scene scene;
  vio::SceneLoader scene_loader;
  if (!scene_loader.LoadSceneFromConfigFile(option.scene_file_path, scene))
    return -1;
    */

  vio::SceneGenerator generator;
  vio::Scene new_scene;
  generator.GenerateSceneRandom(100, 2, new_scene);

  vio::SceneVisualizer visualizer("simple");
  visualizer.VisualizeScene(new_scene);

  vio::Simulator simulator;
  std::vector<std::vector<Eigen::Vector2d>> feature_pos_each_frame;
  if (!simulator.GenerateFeatureMeasurementsFromTrajectory(
          new_scene, feature_pos_each_frame))
    return -1;

  vio::SceneExporter exporter;
  exporter.WriteSceneToYAMLFile(new_scene, "10_2.yaml");

  return 0;
}

void PrintUsage() {
  std::cout << "Error. Unknown arguments.\n";
  std::cout << "Usage: \n";
  std::cout << "       simulator_app\n";
  std::cout << "            -p, --scene_file_path file path of scene to be "
               "simulated.\n";
}
