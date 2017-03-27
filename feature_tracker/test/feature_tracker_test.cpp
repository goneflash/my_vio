#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

class FeatureTrackerTest : public ::testing::Test {};

TEST(FeatureTrackerTest, TestTwoFrame) {
  ASSERT_TRUE(1);
}

TEST(FeatureTrackerTest, TestTwoFrame1) {
  cv::Mat image0 = cv::imread("test_data/frame0.png");
  cv::Mat image1 = cv::imread("test_data/frame1.png");

  ASSERT_TRUE(image0.data);
  ASSERT_TRUE(image1.data);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
}
