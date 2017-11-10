#include <chrono>
#include <iostream>
#include <string>
#include <unordered_map>

#include "feature_tracker.hpp"
#include "util.hpp"
#include "visual_inertial_odometry.hpp"

using namespace std;
using namespace cv;
using namespace vio;

class Options {
 public:
  Options() {}
  string path;
  string format;
};

int TestFramesInFolder(Options option) {
  vector<string> images;
  if (!GetImageNamesInFolder(option.path, option.format, images)) return -1;
  if (images.size() < 2) {
    cout << "Error: Find only " << images.size() << " images.\n";
    return -1;
  }
  cout << "Testing with " << images.size() << " images.\n";

  // Track first frame.
  cv::Mat image0 = cv::imread(images[0]);
  std::unique_ptr<vio::ImageFrame> frame_pre(new vio::ImageFrame(image0));

  std::unique_ptr<vio::VisualInertialOdometry> vio;

  // Load camera information.
  const std::string config_file_name = "calibration.yaml";
  const std::string full_path_to_config = option.path + '/' + config_file_name;
  cv::FileStorage config_file;
  vio::CameraModelPtr camera_ptr = nullptr;
  config_file.open(full_path_to_config, cv::FileStorage::READ);
  if (!config_file.isOpened()) {
    std::cout << "Couldn't open config file " << full_path_to_config
              << ". Skipped\n";
  } else {
    camera_ptr = vio::CreateCameraModelFromConfig(config_file["CameraModel"]);
    if (camera_ptr == nullptr) {
      std::cerr << "Error: Couldn't create camera model.\n";
      return -1;
    }
  }

  vio = std::unique_ptr<vio::VisualInertialOdometry>(
      new vio::VisualInertialOdometry(std::move(camera_ptr)));

  vio->Start();

  // Let the vio thread run for a while.
  for (int i = 1; i < images.size(); ++i) {
    cv::Mat image = cv::imread(images[i]);
    vio->ProcessNewImage(image);
    // At 20hz.
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }

  vio->Stop();
#ifdef OPENCV_VIZ_FOUND
  vio->VisualizeCurrentScene();
#endif
  return 0;
}

int main(int argc, char **argv) {
  Options option;
  for (int i = 0; i < argc; ++i) {
    if (!strcmp(argv[i], "-p") || !strcmp(argv[i], "--path")) {
      option.path = argv[++i];
    } else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--format")) {
      option.format = argv[++i];
    }
  }

  if (option.format.size() && option.path.size())
    return TestFramesInFolder(option);

  cout << "Error. Unknown arguments.\n";
  cout << "Usage: \n";
  cout << "       test\n";
  cout << "            -p, --path full_path \n";
  cout << "            -f, --format image format, e.g png, jpg\n";
  cout << "Exampe: \n";
  cout << "./feature_tracker_app -p ~/Project/vio/data/desk_subset/ -f jpg\n";

  return 0;
}
