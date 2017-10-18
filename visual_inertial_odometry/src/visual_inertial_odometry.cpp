#include "visual_inertial_odometry.hpp"

namespace vio {

VisualInertialOdometry::VisualInertialOdometry(CameraModelPtr camera)
    : camera_(camera) {
  // Setup Feature tracker.
  InitializeFeatureTracker();
}

void VisualInertialOdometry::InitializeFeatureTracker() {
  // Create Matcher.
  FeatureMatcherOptions matcher_options;
  std::unique_ptr<FeatureMatcher> matcher =
      FeatureMatcher::CreateFeatureMatcher(matcher_options);

  FeatureTrackerOptions tracker_options;
  // TODO: Select number smartly.
  tracker_options.max_num_feature = 2000;
  feature_tracker_ =
      FeatureTracker::CreateFeatureTracker(tracker_options, std::move(matcher));
}

void VisualInertialOdometry::ProcessNewImage(const cv::Mat &img) {}

}  // vio
