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

  cv::Mat mask(frame.GetImage().size(), CV_8U);
  mask = cv::Scalar(255);

  std::vector<cv::KeyPoint> kp;
  DetectFeatures(frame, kp, mask);

  cv::Mat desc;
  ComputeDescriptors(frame, kp, desc);

  frame.set_keypoints(kp);
  frame.set_descriptors(desc);

  timer.Stop();
  std::cout << "Detect and compute used " << timer.GetInMs() << "ms.\n";
}

void FeatureTracker::GenerateDistributedFeatures(ImageFrame &frame) {
}

}  // vio
