#include "feature_tracker.hpp"

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

}  // vio
