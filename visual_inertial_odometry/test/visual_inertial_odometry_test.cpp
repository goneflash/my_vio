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

    // Setup test data.
    ASSERT_TRUE(GetImageNamesInFolder(
        root_path + "/feature_tracker/test/test_data/long_seq", "jpg", images));
  }

  // Interval is in milliseconds.
  // Images to test.
  void Process(int interval, int num_img_to_test) {
    int count = 0;
    for (auto img : images) {
      cv::Mat image = cv::imread(img);

      vio->ProcessNewImage(image);

      std::this_thread::sleep_for(std::chrono::milliseconds(interval));

      count++;
      if (count == num_img_to_test) break;
    }
  }

  std::vector<std::string> images;
  vio::CameraModelPtr camera;
  std::unique_ptr<vio::VisualInertialOdometry> vio;
};

TEST_F(VisualInertialOdometryTest, Test_Short_Quick) {
  Init();
  vio->Start();

  // 20 ms, 10 frames.
  Process(1, 10);

  vio->Stop();
}

TEST_F(VisualInertialOdometryTest, Test_FullLength_Quick) {
  Init();
  vio->Start();

  // 20 ms, 10 frames.
  Process(1, images.size());

  vio->Stop();
}

TEST_F(VisualInertialOdometryTest, Test_FullLength_NormalSpeed) {
  Init();
  vio->Start();

  // 50 ms, 40 frames.
  Process(30, 40);

  vio->Stop();
}
/*
 * TODO: Will stuck now.
 */
TEST_F(VisualInertialOdometryTest, Test_FullLength_Slow) {
  Init();
  vio->Start();

  // 200 ms, 40 frames.
  Process(200, 40);

  vio->Stop();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
