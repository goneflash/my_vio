#include "gtest/gtest.h"

#include "visual_inertial_odometry.hpp"

#include "config.hpp"
#include "camera_model.hpp"
#include "util.hpp"

class VisualInertialOdometryTest : public ::testing::Test {
 protected:
  void Init() {
    vio::PinholeCameraModel<double>::ParamsArray params;
    // Principal point is camera center.
    params << 1.0, 2.0, 0, 0;
    camera = vio::CameraModelPtr(
        new vio::PinholeCameraModel<double>(480, 640, params));

    vio = std::unique_ptr<vio::VisualInertialOdometry>(
        new vio::VisualInertialOdometry(camera));
  }

  void Process() {
    std::vector<std::string> images;
    ASSERT_TRUE(GetImageNamesInFolder(
        root_path + "/feature_tracker/test/test_data/long_seq", "jpg", images));
    int num_img_to_test = 5;
    for (auto img : images) {
      cv::Mat image = cv::imread(img);

      vio->ProcessNewImage(image);

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  vio::CameraModelPtr camera;
  std::unique_ptr<vio::VisualInertialOdometry> vio;
};

TEST_F(VisualInertialOdometryTest, Test0) {
  Init();
  vio->Start();

  Process();

  vio->Stop();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
