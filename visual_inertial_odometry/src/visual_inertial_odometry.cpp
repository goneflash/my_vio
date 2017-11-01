#include "visual_inertial_odometry.hpp"

#include "multiview.hpp"

namespace vio {

VisualInertialOdometry::VisualInertialOdometry(CameraModelPtr camera)
    : camera_(camera),
      vio_status_(UNINITED),
      last_keyframe_(nullptr),
      running_process_buffer_thread_(false),
      running_initializer_thread_(false),
      num_skipped_frames_(0) {
  // Setup Feature tracker.
  InitializeFeatureTracker();
  InitializeVIOInitializer();
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

void VisualInertialOdometry::InitializeVIOInitializer() {
  vio::MapInitializerOptions options;
  options.method = vio::MapInitializerOptions::NORMALIZED8POINTFUNDAMENTAL;
  options.use_f_ransac = false;

  map_initializer_ = MapInitializer::CreateMapInitializer(options);
}

void VisualInertialOdometry::ProcessNewImage(cv::Mat &img) {
  data_buffer_.AddImageData(img);
}

void VisualInertialOdometry::AddNewKeyframeFromImage() {

}

void VisualInertialOdometry::ProcessDataInBuffer() {
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  for (;;) {
    {
      std::unique_lock<std::mutex> tmp_lock(
          running_process_buffer_thread_mutex_);
      if (!running_process_buffer_thread_) break;
    }

    /* TODO:
     * When processing is faster than coming images and the images has end, it
     * will stuck here.
     */
    cv::Mat new_image;
    if (data_buffer_.GetImageDataOrEndOfBuffer(new_image)) break;

    std::unique_ptr<vio::ImageFrame> frame_cur(new vio::ImageFrame(new_image));

    // TODO: May need to skip the first several frames.
    /*
     * Handle first frame.
     */
    keyframe_lock.lock();
    if (!last_keyframe_) {
      keyframe_lock.unlock();
      // TODO: ImageFrame must have features before passing to a keyframe.
      feature_tracker_->ComputeFrame(*frame_cur);
      std::unique_ptr<Keyframe> first_keyframe =
          std::unique_ptr<Keyframe>(new Keyframe(std::move(frame_cur)));
      // TODO: Should not be added before.
      const KeyframeId tmp_id = first_keyframe->frame_id;

      keyframe_lock.lock();
      keyframes_[first_keyframe->frame_id] = std::move(first_keyframe);
      last_keyframe_ = keyframes_[tmp_id].get();
      keyframe_lock.unlock();

      continue;
    }
    keyframe_lock.unlock();

    /*
     * Run feature matching for new frame.
     */
    std::vector<cv::DMatch> matches;
    // TODO: Return tracking evaluation as well.
    // Should use mutex here. Otherwise, if the pointed keyframe is changed in
    // another thread.
    keyframe_lock.lock();
    feature_tracker_->TrackFrame(*last_keyframe_->image_frame.get(), *frame_cur,
                                 matches);
    keyframe_lock.unlock();
    // std::cout << "Feature number in new frame " <<
    // frame_cur->keypoints().size() << std::endl;
    std::cout << "Found match " << matches.size() << std::endl;

    /*
     * There three cases:
     * 1. Skip
     * 2. Add as new keyframe
     * 3. Lost tracking, need to restart. TODO: or loop closure.
     */
    if (matches.size() > 500 && num_skipped_frames_ < 5) {
      // Robust tracking. Skip this frame.
      // std::cout << "Skipped a frame with " << matches.size() << "
      // matches.\n";
      num_skipped_frames_++;
      continue;
    } else if (matches.size() < 10) {
      // TODO
      std::cout << "Warning: Lost tracking. Restarting...";
    } else {
      num_skipped_frames_ = 0;
      // TODO: Also skip if estimated motion is bad.
      // Add this frame as a new keyframe.
      std::unique_ptr<Keyframe> new_keyframe =
          std::unique_ptr<Keyframe>(new Keyframe(std::move(frame_cur)));

      // Add tracks.
      std::lock(landmarks_lock, keyframe_lock);
      ProcessMatchesToLandmarks(last_keyframe_, new_keyframe.get(), matches,
                                landmarks_);
      landmarks_lock.unlock();
      keyframe_lock.unlock();

      // TODO: Should not be added before.
      keyframe_lock.lock();
      const KeyframeId tmp_id = new_keyframe->frame_id;
      keyframes_[new_keyframe->frame_id] = std::move(new_keyframe);
      last_keyframe_ = keyframes_[tmp_id].get();

      std::cout << "Now total keyframes is: " << keyframes_.size() << std::endl;
      keyframe_lock.unlock();
    }

    /*
     * Choose what to do depend on the status of VIO
     */
    std::unique_lock<std::mutex> initializer_lock(running_initializer_thread_mutex_);
    if (vio_status_ == UNINITED) {
      // TODO: Shouldn't Deadlock here. Because in RunInitialzier it is required
      // to lock both together.
      if (!running_initializer_thread_) {
        running_initializer_thread_ = true;
        // TODO: Should unlock here, or remove, just release when end of this
        // section?
        initializer_lock.unlock();

        // Run initilizer on the most recent frames.
        std::lock(landmarks_lock, keyframe_lock);
        std::vector<std::vector<cv::Vec2d> > feature_vectors;
        std::vector<KeyframeId> frame_ids;
        CopyDataForInitializer(landmarks_, keyframes_, frame_ids,
                               feature_vectors);
        std::cout << "Prepared for initialization:\n"
                  << "Total frames : " << keyframes_.size()
                  << "\nTotal features: " << feature_vectors[0].size() << "\n";
        landmarks_lock.unlock();
        keyframe_lock.unlock();

        // Run independently. If succeeded, write results to the frames and
        // initialize landmarks.
        // TODO: Should use future::state::ready.
        initializer_lock.lock();
        if (initializer_thread_ != nullptr && initializer_thread_->joinable())
          initializer_thread_->join();
        initializer_lock.unlock();
        // TODO: The problem is if already called Stop. Then this should not
        // execute.
        // TODO: Doesn't make sense, because need to check every step in this
        // thread
        // if it has already received signal to stop!!!
        {
          std::unique_lock<std::mutex> tmp_lock(
              running_process_buffer_thread_mutex_);
          if (!running_process_buffer_thread_) break;
        }
        initializer_thread_ = std::unique_ptr<std::thread>(
            new std::thread(&VisualInertialOdometry::RunInitializer, this,
                            frame_ids, feature_vectors));
      } else {  // already running a initialization thread.
        initializer_lock.unlock();
      }
    } else {
        initializer_lock.unlock();
      // Estmiate the pose of current frame.
    }
  }
}

void VisualInertialOdometry::RunInitializer(
    const std::vector<KeyframeId> &frame_ids,
    const std::vector<std::vector<cv::Vec2d> > &feature_vectors) {
  std::vector<cv::Point3f> points3d;
  std::vector<bool> points3d_mask;
  std::vector<cv::Mat> Rs_est, ts_est;

  // TODO: Change this.
  cv::Matx33d K_ = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);
  if (!map_initializer_->Initialize(feature_vectors, cv::Mat(K_), points3d,
                                    points3d_mask, Rs_est, ts_est)) {
    std::cerr << "Warning: Initialization failed.\n\n";
    // TODO: Clear all keyframes and landmarks.
  } else {
    std::cerr << "Initialization Success.\n\n";
    CopyInitializedFramesAndLandmarksData(frame_ids, Rs_est, ts_est);
      vio_status_ = INITED;
  }

  // TODO: Consider using future / promises.
  // It should be fine here, because tmp_lock won't release until the function
  // is over and the thread is finished.
  std::unique_lock<std::mutex> tmp_lock(running_initializer_thread_mutex_);
  running_initializer_thread_ = false;
}

void VisualInertialOdometry::CopyInitializedFramesAndLandmarksData(
    const std::vector<KeyframeId> &frame_ids,
    const std::vector<cv::Mat> &Rs_est, const std::vector<cv::Mat> &ts_est) {
  // TODO: Check size equal: frame_ids, Rs_est, ts_est.
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);

  keyframe_lock.lock();
  for (int i = 0; i < frame_ids.size(); ++i) {
    keyframes_[frame_ids[i]]->SetPose(Rs_est[i], ts_est[i]);
  }
  keyframe_lock.unlock();

  // Triangluate features again.
  // TODO: Although it's now duplicated with the one in MapInitializer, it should have better method in the future.

  // Calculate poses for current keyframes.

  // std::lock(landmarks_lock, keyframe_lock);
}

#ifdef OPENCV_VIZ_FOUND
void VisualInertialOdometry::VisualizeCurrentScene() {
  // TODO: I think the problem using some flag is that in the whole process
  // the flag might change. So must make sure the change of flag won't affect
  // the current thread. For example here if vio_status_is changed after here,
  // e.g. whole scene is reset, then it will be troublsome.
  // TODO: For debug reason, here just hold the status mutex for the whole time.
  if (vio_status_ != INITED) {
    std::cerr << "Error: VIO not initialized. Couldn't visualize.\n";
    return;
  }

  vio::Scene scene;
  vio::SceneVisualizer visualizer("simple");

  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_);
  for (const auto &id_to_frame : keyframes_) {
    const auto &keyframe = id_to_frame.second;
    if (keyframe->inited_pose_) {
      std::cout << "frame " << id_to_frame.first.id() << " is inited.\n";
      scene.trajectory.push_back(keyframe->pose);
    } else {
      std::cout << "frame " << id_to_frame.first.id() << " is not inited.\n";
    }
  }

  visualizer.VisualizeScene(scene);
}
#endif

void RemoveUnmatchedFeatures(Keyframe *frame) {
  // TODO
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

void RemoveShortTrackLengthLandmark(LandmarkId landmark_id,
                                    Landmarks &landmarks,
                                    Keyframes &keyframes) {}

void RemoveShortTracks(Landmarks &landmarks, Keyframes &keyframes,
                       KeyframeId &cur_keyframe_id) {
  // TODO
}

void CopyDataForInitializer(
    const Landmarks &landmarks, const Keyframes &keyframes,
    std::vector<KeyframeId> &frame_ids,
    std::vector<std::vector<cv::Vec2d> > &feature_vectors) {
  feature_vectors.resize(keyframes.size());
  frame_ids.resize(keyframes.size());
  int frame_count = 0;
  /*
   * Map KeyframeId to the feature_vectors.
   *
   * So for example, there are two frames with id 111 and 222:
   * feature_ids = { 111, 222 }
   * keyframe_id_to_id = { 111 -> 0, 222 -> 1 }
   * feature_vectors = { measurements in 111, measurement in 222 }
   *
   */
  std::unordered_map<KeyframeId, int> keyframe_id_to_id;
  for (const auto &keyframe_ptr : keyframes) {
    frame_ids[frame_count] = keyframe_ptr.second->frame_id;
    keyframe_id_to_id[keyframe_ptr.second->frame_id] = frame_count;
    frame_count++;
  }

  for (const auto &landmark_ptr : landmarks) {
    // TODO: Only add landmarks that are visible to some of the frames.
    // Now only add landmarks that are visible to all frames.
    const Landmark &landmark = *(landmark_ptr.second);
    if (landmark.keyframe_to_feature.size() == keyframes.size()) {
      for (const auto &measurement_ptr : landmark.keyframe_to_feature) {
        const KeyframeId &keyframe_id = measurement_ptr.first;
        if (keyframe_id_to_id.find(keyframe_id) == keyframe_id_to_id.end()) {
          // TODO: Error! Probably didn't clear previous unused Keyframes well.
          std::cerr << "Error: Probably didn't clear previous unused Keyframes "
                       "well.\n";
          return;
        }
        feature_vectors[keyframe_id_to_id[keyframe_id]].push_back(
            cv::Vec2d(measurement_ptr.second.x, measurement_ptr.second.y));
      }
    }
  }
}

}  // vio
