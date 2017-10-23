#include "gtest/gtest.h"

#include "mapdata_types.hpp"


class UniqueIdTest : public ::testing::Test {
  public:
    template <typename T>
    void TestUniqueId() {
      for (int i = 0; i < 10; ++i) {
        T frame_id = vio::CreateNewId<T>();
        ASSERT_EQ(frame_id.id(), i);
      }
    }

    template <typename T>
    void TestCopyId() {
      T id = vio::CreateNewId<T>();
      T new_id = id;
      ASSERT_EQ(id.id(), new_id.id());
    }
};

TEST_F(UniqueIdTest, TestUniqueKeyframeId) {
  TestUniqueId<vio::KeyframeId>();
  TestCopyId<vio::KeyframeId>();
}

TEST_F(UniqueIdTest, TestUniqueLandmarkId) {
  TestUniqueId<vio::LandmarkId>();
  TestCopyId<vio::LandmarkId>();
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
