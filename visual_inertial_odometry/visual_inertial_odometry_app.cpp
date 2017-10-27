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

  vio::CameraModelPtr camera;
  std::unique_ptr<vio::VisualInertialOdometry> vio;

  vio::PinholeCameraModel<double>::ParamsArray params;
  // Principal point is camera center.
  params << 1.0, 2.0, 0, 0;
  camera = vio::CameraModelPtr(
      new vio::PinholeCameraModel<double>(480, 640, params));

  vio = std::unique_ptr<vio::VisualInertialOdometry>(
      new vio::VisualInertialOdometry(camera));

  vio->Start();

  // Let the vio thread run for a while.
  for (int i = 1; i < images.size(); ++i) {
    cv::Mat image = cv::imread(images[i]);
    vio->ProcessNewImage(image);
    // At 20hz.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
