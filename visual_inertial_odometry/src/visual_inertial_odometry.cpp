#include "visual_inertial_odometry.hpp"

namespace vio {

VisualInertialOdometry::VisualInertialOdometry(CameraModelPtr camera)
    : camera_(camera), last_keyframe_(nullptr), vio_status_(UNINITED) {
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
    if (!KeepRunningMainWork()) {
      std::cout << "Image buffer stats:\n";
      data_buffer_.image_buffer_stats().Print();
      LandmarkStats landmark_stats;
      GetLandmarkStats(landmarks_, landmark_stats);
      landmark_stats.Print();
      break;
    }

    cv::Mat new_image = data_buffer_.GetImageData();
    std::unique_ptr<vio::ImageFrame> frame_cur(new vio::ImageFrame(new_image));

    // TODO: May need to skip the first several frames.
    // Handle first frame.
    if (!last_keyframe_) {
      std::unique_ptr<Keyframe> first_keyframe =
          std::unique_ptr<Keyframe>(new Keyframe(std::move(frame_cur)));
      // TODO: Should not be added before.
      const KeyframeId tmp_id = first_keyframe->frame_id;
      keyframes_[first_keyframe->frame_id] = std::move(first_keyframe);
      last_keyframe_ = keyframes_[tmp_id].get();

      continue;
    }

    std::vector<cv::DMatch> matches;
    // TODO: Return tracking evaluation as well.
    feature_tracker_->TrackFrame(*last_keyframe_->image_frame.get(), *frame_cur,
                                 matches);
    std::cout << "Feature number in new frame " << frame_cur->keypoints().size()
              << std::endl;
    std::cout << "Found match " << matches.size() << std::endl;

    /*
     * There three cases:
     * 1. Skip
     * 2. Add as new keyframe
     * 3. Lost tracking, need to restart. TODO: or loop closure.
     */
    if (matches.size() > 1000) {
      // Robust tracking. Skip this frame.
      std::cout << "Skipped a frame with " << matches.size() << " matches.\n";
      continue;
    } else if (matches.size() < 10) {
      std::cout << "Warning: Lost tracking. Restarting...";
    } else {
      // Add this frame as a new keyframe.
      std::unique_ptr<Keyframe> new_keyframe =
          std::unique_ptr<Keyframe>(new Keyframe(std::move(frame_cur)));

      // Add tracks.
      ProcessMatchesToLandmarks(last_keyframe_, new_keyframe.get(), matches,
                                landmarks_);

      // TODO: Should not be added before.
      const KeyframeId tmp_id = new_keyframe->frame_id;
      keyframes_[new_keyframe->frame_id] = std::move(new_keyframe);
      last_keyframe_ = keyframes_[tmp_id].get();
    }

    // TODO: Plot tracking result.
    // cv::imshow("result", new_image);
    // cv::waitKey(20);

    count++;
  }
}

// TODO: How to use it in FeatureTracker to evaluate tracker?
bool ProcessMatchesToLandmarks(Keyframe *pre_frame, Keyframe *cur_frame,
                               const std::vector<cv::DMatch> &matches,
                               Landmarks &landmarks) {
  // TODO: Add test.
  cur_frame->pre_frame_id = pre_frame->frame_id;
  for (const auto match : matches) {
    // TODO: How can landmark_id compared to int value? Default constructor!
    if (pre_frame->features[match.queryIdx].landmark_id == -1) {
      // New landmark.
      std::unique_ptr<Landmark> new_landmark =
          std::unique_ptr<Landmark>(new Landmark());
      new_landmark->AddMeasurementInKeyframe(
          pre_frame->frame_id, pre_frame->features[match.queryIdx].measurement);
      new_landmark->AddMeasurementInKeyframe(
          cur_frame->frame_id, cur_frame->features[match.trainIdx].measurement);
      // Update keyframe feature point to landmark.
      pre_frame->features[match.queryIdx].landmark_id =
          new_landmark->landmark_id;
      cur_frame->features[match.trainIdx].landmark_id =
          new_landmark->landmark_id;
      // Add landmark.
      landmarks[new_landmark->landmark_id] = std::move(new_landmark);
    } else {
      // Handle existing landmark.
      LandmarkId landmark_id = pre_frame->features[match.queryIdx].landmark_id;
      cur_frame->features[match.trainIdx].landmark_id = landmark_id;
      landmarks[landmark_id]->AddMeasurementInKeyframe(
          cur_frame->frame_id, cur_frame->features[match.trainIdx].measurement);
    }
  }
  return true;
}
}  // vio
