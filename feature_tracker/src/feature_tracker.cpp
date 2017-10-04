#include "feature_tracker.hpp"

#include "../../util/include/timer.hpp"

namespace vio {

FeatureTracker *FeatureTracker::CreateFeatureTracker(
    FeatureTrackerOptions option, std::unique_ptr<FeatureMatcher> matcher) {
  switch (option.method) {
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR:
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR:
      return CreateFeatureTrackerOCV(option, std::move(matcher));
    default:
      return nullptr;
  }
}

// Detect features in the entire image.
void FeatureTracker::ComputeFeatures(ImageFrame &frame) {
  Timer timer;
  timer.Start();

  // cv::Mat mask = cv::Mat::zeros(frame.GetImage().size(), CV_8U);
  cv::Mat mask(frame.GetImage().size(), CV_8U);
  mask = cv::Scalar(255);

  // cv::Mat roi(mask, cv::Rect(0, 0, 200, 200));
  // roi = cv::Scalar(255);

  std::vector<cv::KeyPoint> kp;
  DetectFeatures(frame, kp, mask);
  cv::Mat desc;
  ComputeDescriptors(frame, kp, desc);

  frame.set_features(kp, desc);

  timer.Stop();
  std::cout << "Detect and compute used " << timer.GetInMs() << "ms.\n";
}

// TODO: Finish when the odometry is done.
void FeatureTracker::ComputeDistributedFeatures(ImageFrame &frame) {
  // Split the image to |num_bin_col_| x |num_bin_row_| grids.
  const int max_num_feat_per_grid =
      max_num_feature_ / num_bin_col_ / num_bin_row_;

  cv::Mat mask(frame.GetImage().size(), CV_8U);
  cv::Mat roi(mask, cv::Rect(0, 0, 10, 10));
  roi = cv::Scalar(255);
}

}  // vio
