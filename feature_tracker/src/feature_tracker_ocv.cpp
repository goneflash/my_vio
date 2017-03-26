#include "feature_tracker_ocv.hpp"

#include <iostream>

#include "../../util/include/timer.hpp"
#include "feature_matcher_grid_search.hpp"
#include "feature_matcher_ocv.hpp"

namespace vio {

FeatureTracker *FeatureTracker::CreateFeatureTrackerOCV(
    FeatureTrackerOptions option, FeatureMatcher *matcher) {
  switch (option.method) {
    case OCV_BASIC_DETECTOR:
    case OCV_BASIC_DETECTOR_EXTRACTOR:
      return new FeatureTrackerOCV(option, matcher);
    default:
      return nullptr;
  }
}

FeatureTrackerOCV::FeatureTrackerOCV(FeatureTrackerOptions option,
                                     FeatureMatcher *matcher)
    : detector_type_(DETECTORONLY) {
  if (option.detector_type == "ORB") {
    detector_ = cv::ORB::create(option.max_num_feature);
    std::cout << "Created ORB Detector.\n";
  } else if (option.detector_type == "FAST") {
    detector_ = cv::FastFeatureDetector::create();
    std::cout << "Created FAST Detector.\n";
  } else {
    return;
  }

  if (option.detector_type != option.descriptor_type) {
    detector_type_ = DETECTORDESCRIPTOR;
    if (option.descriptor_type == "DAISY") {
      descriptor_ = cv::xfeatures2d::DAISY::create();
      std::cout << "Created DAISY Descriptor.\n";
    } else if (option.descriptor_type == "ORB") {
      descriptor_ = cv::ORB::create();
      std::cout << "Created ORB Descriptor.\n";
    } else {
      return;
    }
  }
  matcher_ = matcher;

  FeatureMatcherOptions long_term_matcher_option;
  long_term_matcher_option.method = OCV;

  long_term_matcher_ =
      FeatureMatcher::CreateFeatureMatcher(long_term_matcher_option);
}

bool FeatureTrackerOCV::TrackFirstFrame(ImageFrame &output_frame) {
  ComputeFeatures(output_frame);
  return true;
}
bool FeatureTrackerOCV::TrackFrame(const ImageFrame &prev_frame,
                                   ImageFrame &new_frame,
                                   std::vector<cv::DMatch> &matches) {
  if (!matcher_) {
    std::cerr << "Error: FeatureMatcher not set up.\n";
    return false;
  }
  ComputeFeatures(new_frame);
  if (!matcher_->Match(prev_frame, new_frame, matches)) return false;

  return true;
}

bool FeatureTrackerOCV::MatchFrame(const ImageFrame &prev_frame,
                                   ImageFrame &new_frame,
                                   std::vector<cv::DMatch> &matches) {
  if (!long_term_matcher_) {
    std::cerr << "Error: Long term FeatureMatcher not set up.\n";
    return false;
  }
  ComputeFeatures(new_frame);
  if (!long_term_matcher_->Match(prev_frame, new_frame, matches)) return false;

  return true;
}

void FeatureTrackerOCV::ComputeFeatures(ImageFrame &frame) {
  Timer timer;
  timer.Start();

  if (detector_type_ == DETECTORONLY) {
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    detector_->detectAndCompute(frame.GetImage(), cv::noArray(), kp, desc);

    frame.set_keypoints(kp);
    frame.set_descriptors(desc);
  } else if (detector_type_ == DETECTORDESCRIPTOR) {
    std::vector<cv::KeyPoint> kp;
    cv::Mat desc;
    detector_->detect(frame.GetImage(), kp);
    descriptor_->compute(frame.GetImage(), kp, desc);

    frame.set_keypoints(kp);
    frame.set_descriptors(desc);
  }

  timer.Stop();
  std::cout << "Detect and compute used " << timer.GetInMs() << "ms.\n";
}

}  // vio
