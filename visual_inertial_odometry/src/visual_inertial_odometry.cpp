#include "visual_inertial_odometry.hpp"

namespace vio {

bool AddFeatureTracks(Keyframe &frame0, Keyframe &frame1,
                      const std::vector<cv::DMatch> &matches,
                      Landmarks &landmarks) {}

VisualInertialOdometry::VisualInertialOdometry(CameraModelPtr camera)
    : camera_(camera) {
  // Setup Feature tracker.
  InitializeFeatureTracker();

  // cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
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

void VisualInertialOdometry::ProcessNewImage(cv::Mat &img) {
  data_buffer_.AddImageData(img);
}

void VisualInertialOdometry::ProcessDataInBuffer() {
  int count = 0;

  cv::Mat first_image = data_buffer_.GetImageData();
  std::unique_ptr<vio::ImageFrame> frame_pre(new vio::ImageFrame(first_image));
  for (;;) {
    if (!KeepRunningMainWork()) break;
    cv::Mat new_image = data_buffer_.GetImageData();

    std::unique_ptr<vio::ImageFrame> frame_cur(new vio::ImageFrame(new_image));
    std::vector<cv::DMatch> matches;
    feature_tracker_->TrackFrame(*frame_pre, *frame_cur, matches);

    std::cout << "Feature number in new frame " << frame_cur->keypoints().size()
              << std::endl;
    std::cout << "Found match " << matches.size() << std::endl;
    frame_pre = std::move(frame_cur);
    // TODO: Plot tracking result.
    // cv::imshow("result", new_image);
    // cv::waitKey(20);

    count++;
    std::cout << "Total image: " << data_buffer_.image_total_num() << std::endl;
    std::cout << "Dropped image: " << data_buffer_.image_dropped_num()
              << std::endl;
    std::cout << "processed " << count << std::endl;
  }
}

}  // vio
