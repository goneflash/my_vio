#include "visual_inertial_odometry.hpp"

namespace vio {

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

    //std::unique_ptr<Keyframe> new_keyframe =
    //    std::unique_ptr<Keyframe>(new Keyframe(std::move());

    count++;
    std::cout << "Total image: " << data_buffer_.image_total_num() << std::endl;
    std::cout << "Dropped image: " << data_buffer_.image_dropped_num()
              << std::endl;
    std::cout << "processed " << count << std::endl;
  }
}

// TODO: How to use it in FeatureTracker to evaluate tracker?
bool ConstructAndAddKeyframe(Keyframe &pre_frame, Keyframe &cur_frame,
                             const std::vector<cv::DMatch> &matches,
                             Landmarks &landmarks) {
  // TODO: Add test.
  cur_frame.pre_frame_id = pre_frame.frame_id;
  for (const auto match : matches) {
    // TODO: How can landmark_id compared to int value? Default constructor!
    if (pre_frame.features[match.queryIdx].landmark_id == -1) {
      // New landmark.
      std::unique_ptr<Landmark> new_landmark =
          std::unique_ptr<Landmark>(new Landmark());
      new_landmark->AddMeasurementInKeyframe(
          pre_frame.frame_id, pre_frame.features[match.queryIdx].measurement);
      new_landmark->AddMeasurementInKeyframe(
          cur_frame.frame_id, cur_frame.features[match.trainIdx].measurement);
      // Update keyframe feature point to landmark.
      pre_frame.features[match.queryIdx].landmark_id =
          new_landmark->landmark_id();
      cur_frame.features[match.trainIdx].landmark_id =
          new_landmark->landmark_id();
      // Add landmark.
      landmarks[new_landmark->landmark_id()] = std::move(new_landmark);
    } else {
      // Handle existing landmark.
      LandmarkId landmark_id = pre_frame.features[match.queryIdx].landmark_id;
      cur_frame.features[match.trainIdx].landmark_id = landmark_id;
      landmarks[landmark_id]->AddMeasurementInKeyframe(
          cur_frame.frame_id, cur_frame.features[match.trainIdx].measurement);
    }
  }
  return true;
}
}  // vio
