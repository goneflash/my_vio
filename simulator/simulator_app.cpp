#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "camera_model.hpp"
#include "simulator.hpp"

void WriteFeatureTracksToFile(
    std::vector<std::vector<Eigen::Vector2d>> &feature_pos_each_frame,
    const std::string &file_name) {
}

int main(int argc, char **argv) {
  vio::Simulator simulator;

  vio::PinholeCameraModel<double>::ParamsArray params;
  params << 1.0, 2.0, 0, 0;
  vio::PinholeCameraModel<double> camera(480, 640, params);


  return 0;
}
