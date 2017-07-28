#include "gtest/gtest.h"

#include "camera_model.hpp"

class CameraModelTest : public ::testing::Test {
 protected:
  template<class CameraModel>
  void TestProjectPoint(const CameraModel &camera,
                        const Eigen::Vector3d &point,
                        const Eigen::Vector2d &expected_pixel,
                        bool expected_success) {
    Eigen::Vector2d pixel;
    camera.image_height();
    bool success = camera.ProjectPoint(point, pixel);
    ASSERT_EQ(success, expected_success);
    if (!expected_success)
      return;
    ASSERT_TRUE(pixel.isApprox(expected_pixel));
  }
};

TEST_F(CameraModelTest, TestPinholeCamera) {
  vio::PinholeCameraModel<double>::ParamsArray params;
  params << 1.0, 2.0, 3.0, 4.0;
  vio::PinholeCameraModel<double> camera(480, 640, params);

  TestProjectPoint(camera, Eigen::Vector3d(0, 0, 1), Eigen::Vector2d(3, 4),
                   true);
  /*
   * x = 1 * 1 + 3 = 4
   * y = 2 * 1 + 4 = 6
   */
  TestProjectPoint(camera, Eigen::Vector3d(1, 1, 1), Eigen::Vector2d(4, 6),
                   true);

  // Behind the camera.
  TestProjectPoint(camera, Eigen::Vector3d(1, 1, -1), Eigen::Vector2d(0, 0),
                   false);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
}
