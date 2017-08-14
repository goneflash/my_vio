#include <opencv2/opencv.hpp>

#include "gtest/gtest.h"

#include "feature_tracker.hpp"
#include "util.hpp"

class FeatureTrackerTest : public ::testing::Test {
 protected:
  void CreateTracker() {
    feature_matcher_ =
        vio::FeatureMatcher::CreateFeatureMatcher(feature_matcher_option_);
    ASSERT_TRUE(feature_matcher_.get() != NULL);

    feature_tracker_ = vio::FeatureTracker::CreateFeatureTracker(
      feature_tracker_option_, std::move(feature_matcher_));
    ASSERT_TRUE(feature_tracker_ != NULL);
  }

  void CreateTwoImageTestData() {
    cv::Mat image0 = cv::imread("../../feature_tracker/test/test_data/close/frame0.png");
    cv::Mat image1 = cv::imread("../../feature_tracker/test/test_data/close/frame1.png");
    ASSERT_TRUE(image0.data);
    ASSERT_TRUE(image1.data);
    std::unique_ptr<vio::ImageFrame> frame0(new vio::ImageFrame(image0));
    std::unique_ptr<vio::ImageFrame> frame1(new vio::ImageFrame(image1));
    frames_.push_back(std::move(frame0));
    frames_.push_back(std::move(frame1));
  }

  void CreateLongSequenceTestData() {
    std::vector<std::string> images;
    ASSERT_TRUE(GetImageNamesInFolder(
          "../../feature_tracker/test/test_data/long_seq", "jpg", images));
    int num_img_to_test = 5;
    for (auto img : images) {
      cv::Mat image = cv::imread(img);
      ASSERT_TRUE(image.data);
      std::unique_ptr<vio::ImageFrame> frame(new vio::ImageFrame(image));
      frames_.push_back(std::move(frame));

      if (frames_.size() == num_img_to_test) {
        break;
      }
    }
  }

  vio::FeatureMatcherOptions feature_matcher_option_;
  std::unique_ptr<vio::FeatureMatcher> feature_matcher_;

  vio::FeatureTrackerOptions feature_tracker_option_;
  vio::FeatureTracker *feature_tracker_;

  std::vector<std::unique_ptr<vio::ImageFrame>> frames_;
};

TEST_F(FeatureTrackerTest, TestTwoFrameDefault_ORB_DAISY) {
  // Use default setup.
  CreateTracker();
  CreateTwoImageTestData();

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames_[0], *frames_[1], matches));
  ASSERT_GE(frames_[0]->keypoints().size(), 200);
  ASSERT_EQ(matches.size(), 1082);
}

TEST_F(FeatureTrackerTest, TestTwoFrameOCV_ORB_DAISY) {
  feature_matcher_option_.method = vio::FeatureMatcherOptions::OCV;
  CreateTracker();
  CreateTwoImageTestData();

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames_[0], *frames_[1], matches));
  ASSERT_EQ(frames_[0]->keypoints().size(), 3037);
  ASSERT_EQ(matches.size(), 1092);
}

TEST_F(FeatureTrackerTest, TestTwoFrameORBPipeline) {
  feature_tracker_option_.method = vio::FeatureTrackerOptions::OCV_BASIC_DETECTOR;
  CreateTracker();
  CreateTwoImageTestData();

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames_[0], *frames_[1], matches));
  ASSERT_EQ(frames_[0]->keypoints().size(), 3037);
  //ASSERT_EQ(matches.size(), 1608);
}

TEST_F(FeatureTrackerTest, TestTwoFrameFAST_DAISY) {
  feature_tracker_option_.method =
      vio::FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR;
  feature_tracker_option_.detector_type = "FAST";
  feature_tracker_option_.descriptor_type = "DAISY";
  CreateTracker();
  CreateTwoImageTestData();

  std::vector<cv::DMatch> matches;
  ASSERT_TRUE(feature_tracker_->TrackFrame(*frames_[0], *frames_[1], matches));
  ASSERT_GE(frames_[0]->keypoints().size(), 200);
  ASSERT_GE(matches.size(), 50);
}

TEST_F(FeatureTrackerTest, TestLongSequenceFAST_DAISY) {
  feature_tracker_option_.method =
      vio::FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR;
  feature_tracker_option_.detector_type = "FAST";
  feature_tracker_option_.descriptor_type = "DAISY";
  CreateTracker();
  CreateLongSequenceTestData();

  for (int i = 1; i < frames_.size(); ++i) {
    std::vector<cv::DMatch> matches;
    ASSERT_TRUE(feature_tracker_->TrackFrame(*frames_[i - 1], *frames_[i], matches));
  }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
      return RUN_ALL_TESTS();
}
