#include "feature_tracker.hpp"

namespace vio {

FeatureTracker *FeatureTracker::CreateFeatureTracker(
    FeatureTrackerOptions option, FeatureMatcher *matcher) {
  switch (option.method) {
    case OCV_BASIC_DETECTOR:
    case OCV_BASIC_DETECTOR_EXTRACTOR:
      return CreateFeatureTrackerOCV(option, matcher);
    default:
      return nullptr;
  }
}

}  // vio
