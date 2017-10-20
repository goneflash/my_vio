#include "gtest/gtest.h"

#include "visual_inertial_odometry.hpp"

#include "config.hpp"

class VisualInertialOdometryTest : public ::testing::Test {
 protected:

  std::unique_ptr<vio::VisualInertialOdometry> vio;
};

TEST_F(VisualInertialOdometryTest, Test0) {
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
