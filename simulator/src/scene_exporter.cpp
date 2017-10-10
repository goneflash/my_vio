#include "scene_exporter.hpp"

#include "opencv2/core.hpp"

namespace vio {

bool SceneExporter::WriteSceneToYAMLFile(const Scene &scene,
                                         const std::string file_name) {
  // Create file.
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  // Write Landmarks.
  fs << "Landmarks"
     << "{";
  {
    fs << "NumLandmarks" << (int)scene.landmarks.size();
    cv::Mat positions(scene.landmarks.size(), 3, CV_64F);
    int i = 0;
    for (const auto &landmark : scene.landmarks) {
      positions.at<double>(0, i) = landmark.position[0];
      positions.at<double>(1, i) = landmark.position[1];
      positions.at<double>(2, i) = landmark.position[2];
      i++;
    }
    fs << "Positions" << positions;
  }
  fs << "}";

  // Write Camera Views
  fs << "CameraPoses"
     << "{";
  {
    fs << "NumPoses" << (int)scene.trajectory.size();
    fs << "Poses"
       << "[";
    {
      for (const auto &view : scene.trajectory) {
        fs << view.R;
        fs << view.t;
      }
    }
    fs << "]";
  }
  fs << "}";

  return true;
}

}  // vio
