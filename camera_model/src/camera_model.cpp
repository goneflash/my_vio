#include "camera_model.hpp"

namespace vio {

CameraModelPtr CreateCameraModelFromConfig(const cv::FileNode &node) {
  const std::string camera_type = (std::string)node["CameraType"];
  const int num_params = (int)node["NumParams"];
  const int image_width = (int)node["ImageWidth"];
  const int image_height = (int)node["ImageHeight"];
    // Load parameters.
    cv::Mat params_cv;
    node["Params"] >> params_cv;

  if (camera_type == "Pinhole") {
    PinholeCameraModel<double>::ParamsArray params;
    // TODO: Check size of params == num_params.
    for (int i = 0; i < num_params; ++i) {
      params[i] = params_cv.at<double>(i);
      std::cout << "Param " << i << " : " << params[i] << std::endl;
    }

    std::unique_ptr<vio::CameraModel> camera =
        std::unique_ptr<vio::CameraModel>(
            new vio::PinholeCameraModel<double>(image_height, image_width, params));
  } else if (camera_type == "

  } else {
    std::cerr << "Error: Unknown camera model name <" << camera_type << ">.\n";
    return nullptr;
  }
}

}  // namespace vio
