#include "gtest/gtest.h"

#include "camera_model.hpp"

class CameraModelTest : public ::testing::Test {
 protected:
  template<class CameraModel>
  void TestProjectPoint(CameraModel &camera,
                        const Eigen::Vector3d &point,
                        const Eigen::Vector2d &expected_pixel) {
    Eigen::Vector2d pixel;
    camera.image_height();
    //camera.ProjectPoint(point, pixel);
  }
};

TEST_F(CameraModelTest, TestPinholeCamera) {
  vio::PinholeCameraModel<double>::ParamsArray params;
  params << 1.0, 2.0, 3.0, 4.0;
  vio::PinholeCameraModel<double> camera(480, 640, params);

  TestProjectPoint(camera, Eigen::Vector3d(0, 0, 1), Eigen::Vector2d(3, 4));
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
}
