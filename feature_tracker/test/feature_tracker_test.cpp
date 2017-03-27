#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

#include "feature_tracker.hpp"

class FeatureTrackerTest : public ::testing::Test {
 protected:
  void CreateTracker() {
    feature_matcher_ =
        vio::FeatureMatcher::CreateFeatureMatcher(feature_matcher_option_);
    ASSERT_TRUE(feature_matcher_ != NULL);

    feature_tracker_ = vio::FeatureTracker::CreateFeatureTracker(
      feature_tracker_option_, std::move(feature_matcher_));
    ASSERT_TRUE(feature_tracker_ != NULL);
  }

  void CreateTwoImageTestData() {
    cv::Mat image0 = cv::imread("../feature_tracker/test/test_data/frame0.png");
    cv::Mat image1 = cv::imread("../feature_tracker/test/test_data/frame1.png");
    ASSERT_TRUE(image0.data);
    ASSERT_TRUE(image1.data);
    std::unique_ptr<vio::ImageFrame> frame0(new vio::ImageFrame(image0));
    std::unique_ptr<vio::ImageFrame> frame1(new vio::ImageFrame(image1));
    frames.push_back(std::move(frame0));
    frames.push_back(std::move(frame1));
  }

  vio::FeatureMatcherOptions feature_matcher_option_;
  std::unique_ptr<vio::FeatureMatcher> feature_matcher_;

  vio::FeatureTrackerOptions feature_tracker_option_;
  vio::FeatureTracker *feature_tracker_;

  std::vector<std::unique_ptr<vio::ImageFrame>> frames;
};

TEST_F(FeatureTrackerTest, TestTwoFrameDefault_ORB_DAISY) {
  // Use default setup.
  CreateTracker();
  CreateTwoImageTestData();

  ASSERT_TRUE(feature_tracker_->TrackFirstFrame(*frames[0]));
  ASSERT_EQ(frames[0]->keypoints().size(), 3037);

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames[0], *frames[1], matches));
  ASSERT_EQ(matches.size(), 1063);
}

TEST_F(FeatureTrackerTest, TestTwoFrameOCV_ORB_DAISY) {
  feature_matcher_option_.method = vio::FeatureMatcherOptions::OCV;
  CreateTracker();
  CreateTwoImageTestData();

  ASSERT_TRUE(feature_tracker_->TrackFirstFrame(*frames[0]));
  ASSERT_EQ(frames[0]->keypoints().size(), 3037);

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames[0], *frames[1], matches));
  ASSERT_EQ(matches.size(), 1092);
}

TEST_F(FeatureTrackerTest, TestTwoFrameORBPipeline) {
  feature_tracker_option_.method = vio::FeatureTrackerOptions::OCV_BASIC_DETECTOR;
  CreateTracker();
  CreateTwoImageTestData();

  ASSERT_TRUE(feature_tracker_->TrackFirstFrame(*frames[0]));
  ASSERT_EQ(frames[0]->keypoints().size(), 3037);

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames[0], *frames[1], matches));
  //ASSERT_EQ(matches.size(), 1608);
}

TEST_F(FeatureTrackerTest, TestTwoFrameFAST_DAISY) {
  feature_tracker_option_.method =
      vio::FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR;
  feature_tracker_option_.detector_type = "FAST";
  feature_tracker_option_.descriptor_type = "DAISY";
  CreateTracker();
  CreateTwoImageTestData();

  ASSERT_TRUE(feature_tracker_->TrackFirstFrame(*frames[0]));
  ASSERT_EQ(frames[0]->keypoints().size(), 3812);

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames[0], *frames[1], matches));
  ASSERT_EQ(matches.size(), 2254);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
}
