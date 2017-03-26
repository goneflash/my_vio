#include "gtest/gtest.h"

class FeatureTrackerTest : public ::testing::Test {};

TEST(FeatureTrackerTest, TestTwoFrame) {
  ASSERT_NE(20, 10);
  ASSERT_EQ(20, 20);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
}
