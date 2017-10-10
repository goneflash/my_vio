#include "scene_exporter.hpp"

#include "opencv2/core.hpp"

#include "camera_model.hpp"

namespace vio {

bool SceneExporter::WriteSceneToYAMLFile(const Scene &scene,
                                         const std::string file_name) {
  // Create file.
  cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
  // Write Camera Model.
  fs << "CameraModel"
     << "{";
  {
    fs << "CameraType";
    switch (scene.camera->camera_model_type()) {
      case PINHOLE: {
        fs << "Pinhole";
        fs << "NumParams" << 4;
        Eigen::Array<double, 4, 1> params =
            reinterpret_cast<PinholeCameraModel<double> *>(scene.camera.get())
                ->params();
        cv::Mat p(1, 4, CV_64F);
        p.at<double>(0) = params[0];
        p.at<double>(1) = params[1];
        p.at<double>(2) = params[2];
        p.at<double>(3) = params[3];
        fs << "Params" << p;
        break;
      }
      default:
        std::cerr << "Error exporting scene: Unknown camera type.\n";
        return false;
    }
  }
  fs << "}";
  // Write Landmarks.
  fs << "Landmarks"
     << "{";
  {
    fs << "NumLandmarks" << (int)scene.landmarks.size();
    cv::Mat positions(scene.landmarks.size(), 3, CV_64F);
    int i = 0;
    for (const auto &landmark : scene.landmarks) {
      positions.at<double>(i, 0) = landmark.position[0];
      positions.at<double>(i, 1) = landmark.position[1];
      positions.at<double>(i, 2) = landmark.position[2];
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
        fs << "{";
        fs << "R" << view.R;
        fs << "t" << view.t;
        fs << "}";
      }
    }
    fs << "]";
  }
  fs << "}";

  return true;
}

}  // vio
