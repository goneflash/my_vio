#include "feature_tracker_ocv.hpp"

#include <iostream>

#include "../../util/include/timer.hpp"
#include "feature_matcher_grid_search.hpp"
#include "feature_matcher_ocv.hpp"

namespace vio {

FeatureTracker *FeatureTracker::CreateFeatureTrackerOCV(
    FeatureTrackerOptions option, std::unique_ptr<FeatureMatcher> matcher) {
  switch (option.method) {
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR:
    case FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR:
      return new FeatureTrackerOCV(option, std::move(matcher));
    default:
      return nullptr;
  }
}

FeatureTrackerOCV::FeatureTrackerOCV(FeatureTrackerOptions option,
                                     std::unique_ptr<FeatureMatcher> matcher)
    : detector_type_(DETECTORONLY), max_feature_per_frame_(option.max_num_feature) {
  if (option.detector_type == "ORB") {
    // Parameters:
    // int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31,
    // int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE,
    // int patchSize=31, int fastThreshold=20
    detector_ = cv::ORB::create(max_feature_per_frame_);
    std::cout << "Created ORB Detector.\n";
  } else if (option.detector_type == "FAST") {
    // Parameters:
    // int threshold=10
    // bool nonmaxSuppression=true
    // int type=FastFeatureDetector::TYPE_9_16
    detector_ = cv::FastFeatureDetector::create();
    std::cout << "Created FAST Detector.\n";
  } else if (option.detector_type == "SURF") {
    detector_ = cv::xfeatures2d::SURF::create();
    std::cout << "Created SURF Detector.\n";
  } else if (option.detector_type == "SIFT") {
    detector_ = cv::xfeatures2d::SIFT::create();
    std::cout << "Created SIFT Detector.\n";
  } else {
    return;
  }
  if (detector_ == NULL) {
    std::cerr << "Error: Unable to create detector.\n";
    return;
  }

  // If use different detector and descriptor, must explicitly specify.
  if (option.method == FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR &&
      option.detector_type == option.descriptor_type) {
    std::cerr << "Error: Same detector for detector and descriptor.\n";
    return;
  }

  if (option.method == FeatureTrackerOptions::OCV_BASIC_DETECTOR_EXTRACTOR ||
      option.detector_type != option.descriptor_type) {
    detector_type_ = DETECTORDESCRIPTOR;
    if (option.descriptor_type == "DAISY") {
      descriptor_ = cv::xfeatures2d::DAISY::create();
      std::cout << "Created DAISY Descriptor.\n";
    } else if (option.descriptor_type == "ORB") {
      // TODO: Add argument
      descriptor_ = cv::ORB::create();
      std::cout << "Created ORB Descriptor.\n";
    } else if (option.descriptor_type == "FREAK") {
      descriptor_ = cv::xfeatures2d::FREAK::create();
      std::cout << "Created FREAK Descriptor.\n";
    } else if (option.descriptor_type == "SURF") {
      descriptor_ = cv::xfeatures2d::SURF::create();
      std::cout << "Created SURF Descriptor.\n";
    } else if (option.descriptor_type == "SIFT") {
      descriptor_ = cv::xfeatures2d::SIFT::create();
      std::cout << "Created SIFT Descriptor.\n";
    } else {
      return;
    }
    if (descriptor_ == NULL) {
      std::cerr << "Error: Unable to create descriptor.\n";
      return;
    }
  }
  if (matcher == NULL) {
    std::cerr << "Error: No matcher provided to tracker.\n";
    return;
  }
  matcher_ = std::move(matcher);

  FeatureMatcherOptions long_term_matcher_option;
  long_term_matcher_option.method = FeatureMatcherOptions::OCV;

  std::cout << "Creating long term matcher.\n";
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

  Timer timer;
  timer.Start();

  if (!matcher_->Match(prev_frame, new_frame, matches)) return false;

  timer.Stop();
  std::cout << "Matching used " << timer.GetInMs() << "ms.\n";

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
