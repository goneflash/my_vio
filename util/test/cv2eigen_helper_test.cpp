#include "gtest/gtest.h"

#include "cv2eigen_helper.hpp"

/*
class CV2EigenHelperTest : public ::testing::Test {
};
*/

TEST(CV2EigenHelperTest, Eigen_vec3d_2_cv_vec3d) {
  Eigen::Vector3d vec_in(1.1, 2.2, 3.3);
  cv::Vec3d vec_out = EigenVec3dToCVVec3d(vec_in);
  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(vec_in[i], vec_out[i]);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
