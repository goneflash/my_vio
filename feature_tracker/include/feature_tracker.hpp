#ifndef VIO_FEATURE_TRACKER_
#define VIO_FEATURE_TRACKER_

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

#include "feature_matcher.hpp"
#include "feature_tracker_options.hpp"
#include "image_frame.hpp"

namespace vio {

class FeatureTracker {
 public:
  FeatureTracker() {}
  ~FeatureTracker() {}

  static FeatureTracker *CreateFeatureTracker(FeatureTrackerOptions option,
                                              FeatureMatcher *matcher);

  static FeatureTracker *CreateFeatureTrackerOCV(FeatureTrackerOptions option,
                                                 FeatureMatcher *matcher);

  // TODO: Could change to const.
  virtual bool TrackFirstFrame(ImageFrame &output_frame) = 0;
  // TODO: Might need to use customized Match class.
  virtual bool TrackFrame(const ImageFrame &prev_frame,
                          ImageFrame &output_frame,
                          std::vector<cv::DMatch> &matches) = 0;
  virtual bool MatchFrame(const ImageFrame &prev_frame,
                          ImageFrame &output_frame,
                          std::vector<cv::DMatch> &matches) = 0;

 protected:
  FeatureMatcher *matcher_;
  FeatureMatcher *long_term_matcher_;
};

}  // vio

#endif
