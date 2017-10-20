#include "gtest/gtest.h"

#include "visual_inertial_odometry.hpp"

#include "config.hpp"
#include "camera_model.hpp"

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
  vio::CameraModelPtr camera;
  std::unique_ptr<vio::VisualInertialOdometry> vio;
};

TEST_F(VisualInertialOdometryTest, Test0) {
  Init();
  vio->Start();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
