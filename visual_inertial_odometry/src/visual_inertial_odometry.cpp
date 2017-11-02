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

void VisualInertialOdometry::ProcessDataInBuffer() {
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  for (;;) {
    if (!running_process_buffer_thread_) break;

    cv::Mat new_image;
    if (data_buffer_.GetImageDataOrEndOfBuffer(new_image)) break;
    if (!AddNewKeyframeFromImage(new_image)) {
      continue;
    }
    // Only initialize if there're more than two keyframes.
    keyframe_lock.lock();
    if (keyframes_.size() < 2) {
      keyframe_lock.unlock();
      continue;
    }
    keyframe_lock.unlock();

    /*
     * Choose what to do depend on the status of VIO
     */
    if (vio_status_ == UNINITED) {
      // TODO: Shouldn't Deadlock here. Because in RunInitialzier it is required
      // to lock both together.
      if (!running_initializer_thread_) {
        running_initializer_thread_ = true;

        // Run initilizer on the most recent frames.
        std::lock(landmarks_lock, keyframe_lock);
        std::vector<std::vector<cv::Vec2d> > feature_vectors;
        std::vector<KeyframeId> frame_ids;
        // TODO: Currently copy data from first two frames.
        CopyDataForInitializer(landmarks_, keyframes_, frame_ids,
                               feature_vectors);
        std::cout << "Prepared for initialization:\n"
                  << "Total frames : " << keyframes_.size()
                  << "\nTotal features: " << feature_vectors[0].size() << "\n";
        std::cout << "Frames to be initialized: ";
        for (const auto &frame_id : frame_ids)
          std::cout << frame_id.id() << " ";
        std::cout << std::endl;
        landmarks_lock.unlock();
        keyframe_lock.unlock();

        // Run independently. If succeeded, write results to the frames and
        // initialize landmarks.

        /*
        running_initializer_flag_ =
            std::async(&VisualInertialOdometry::RunInitializer, this, frame_ids,
                       feature_vectors);
        */

        // TODO: Should use future::state::ready.
        if (initializer_thread_ != nullptr && initializer_thread_->joinable())
          initializer_thread_->join();
        // TODO: Doesn't make sense, because need to check every step in this
        // thread if it has already received signal to stop!!!
        if (!running_process_buffer_thread_) return;
        initializer_thread_ = std::unique_ptr<std::thread>(
            new std::thread(&VisualInertialOdometry::RunInitializer, this,
                            frame_ids, feature_vectors));
        std::cout << "Initializer thread started ...\n";
      } else {
        std::cout << "Not initialized, but already running a initialization "
                     "thread ...\n";
      }
    } else {
      // Estmiate the pose of current frame.
      // TODO: Wait until all keyframes are pose_inited.
    }
  }
}

bool VisualInertialOdometry::AddNewKeyframeFromImage(const cv::Mat &new_image) {
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);

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
    std::cout << "Last keyframe points to " << tmp_id.id() << std::endl;
    keyframe_lock.unlock();
    return true;
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
  // Not matched features are useless since we only match consecutive frames.
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

  // TODO: Should not do this because the next frame will be far
  // if the feature processing takes long time.
  if (matches.size() > 600 && num_skipped_frames_ < 3) {
    // Robust tracking. Skip this frame.
    // std::cout << "Skipped a frame with " << matches.size() << "
    // matches.\n";
    num_skipped_frames_++;
    return false;
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
    ProcessMatchesAndAddToLandmarks(last_keyframe_, new_keyframe.get(), matches,
                                    landmarks_);
    RemoveUnmatchedFeatures(*last_keyframe_);
    landmarks_lock.unlock();
    keyframe_lock.unlock();

    // TODO: Should not be added before.
    keyframe_lock.lock();
    const KeyframeId &tmp_id = new_keyframe->frame_id;
    keyframes_[new_keyframe->frame_id] = std::move(new_keyframe);
    last_keyframe_ = keyframes_[tmp_id].get();

    std::cout << "Last keyframe points to " << tmp_id.id() << std::endl;
    std::cout << "Now total keyframes is: " << keyframes_.size() << std::endl;
    keyframe_lock.unlock();
  }
  return true;
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
    // Probably want to only remove the keyframes that failed initialization.
    // There are other cases, e.g. loop closure, lost tracking.
    for (auto &frame_id : frame_ids) {
      if (!RemoveKeyframe(frame_id)) {
        std::cout << "Fatal Error: Failed to remove keyframe.\n";
      }
      std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_);
      std::cout << "Removed keyframes. Now " << keyframes_.size() << " left.\n";
    }
  } else {
    std::cerr << "Initialization Success.\n\n";
    CopyInitializedFramesAndLandmarksData(frame_ids, Rs_est, ts_est);
    vio_status_ = INITED;
  }

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

  // TODO: For now, only support two frames.
  if (frame_ids.size() != 2) {
    std::cout << "Nononono...only 2 frames.\n";
    return;
  }

  // TODO: Change this.
  cv::Matx33d K = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);

  // Triangluate landmarks.
  // TODO: Although it's now duplicated with the one in MapInitializer, it
  // should have better method in the future.
  std::lock(landmarks_lock, keyframe_lock);
  int good_count = 0, tested_count = 0;
  for (auto &landmark_ptr : landmarks_) {
    std::vector<cv::Vec2d> kp;
    std::vector<cv::Mat> P, R, t;
    for (auto &frame_id : frame_ids) {
      const auto &ptr = landmark_ptr.second->keyframe_to_feature.find(frame_id);
      if (ptr == landmark_ptr.second->keyframe_to_feature.end()) continue;
      // TODO: check exists.
      const auto &keyframe_ptr = keyframes_.find(frame_id);
      if (keyframe_ptr == keyframes_.end()) continue;
      const Keyframe &keyframe = *(keyframe_ptr->second);
      if (!keyframe.inited_pose()) continue;

      // Prepare data for triangulation.
      kp.push_back(cv::Vec2d(ptr->second.x, ptr->second.y));
      R.push_back(keyframe.pose.R);
      cv::Mat tmp_t = cv::Mat(3, 1, CV_64F);
      tmp_t.at<double>(0) = keyframe.pose.t[0];
      tmp_t.at<double>(1) = keyframe.pose.t[1];
      tmp_t.at<double>(2) = keyframe.pose.t[2];
      t.push_back(tmp_t);
      cv::Mat tmp_P;
      RtToP(R.back(), t.back(), tmp_P);
      tmp_P = cv::Mat(K) * tmp_P;
      P.push_back(tmp_P);
    }

    // TODO
    if (kp.size() != 2) continue;

    tested_count++;
    // TODO: Assume 2 frames.
    cv::Point3f point_3d;
    TriangulateDLT(kp[0], kp[1], P[0], P[1], point_3d);
    if (IsGoodTriangulatedPoint(kp[0], kp[1], R[0], t[0], R[1], t[1], P[0],
                                P[1], point_3d)) {
      landmark_ptr.second->position[0] = point_3d.x;
      landmark_ptr.second->position[1] = point_3d.y;
      landmark_ptr.second->position[2] = point_3d.z;
      good_count++;
    }
  }
  std::cout << "Tested " << tested_count << " landmarks.\n";
  std::cout << "Triangulated " << good_count << " landmarks.\n";
}

bool VisualInertialOdometry::CalculatePoseForNewKeyframe(Keyframe &new_frame) {
  if (new_frame.pre_frame_id == -1) return false;
  const auto &ptr = keyframes_.find(new_frame.pre_frame_id);
  if (ptr == keyframes_.end()) return false;
  const Keyframe &pre_frame = *(ptr->second);
  if (!pre_frame.inited_pose()) return false;

  // Gather data for estimating the pose.
  std::vector<cv::Point3f> points3d;
  std::vector<cv::Point2f> points2d;
  std::vector<int> points_index;
  // TODO
  for (const auto &match : new_frame.match_to_pre_frame) {
  }

  return true;
}

bool VisualInertialOdometry::TriangulteLandmarksInKeyframes(
    const std::vector<KeyframeId> &frame_ids) {}

bool VisualInertialOdometry::RemoveKeyframe(const KeyframeId &frame_id) {
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);
  std::lock(landmarks_lock, keyframe_lock);

  auto frame_ptr = keyframes_.find(frame_id);
  if (frame_ptr == keyframes_.end()) {
    std::cerr << "Error: Couldn't find the keyframe to remove.\n";
    return false;
  }

  Keyframe *keyframe = frame_ptr->second.get();
  for (auto &feature : keyframe->features) {
    LandmarkId ld = feature.second.landmark_id;
    if (ld == -1) continue;
    auto landmark_ptr = landmarks_.find(ld);
    if (landmark_ptr == landmarks_.end()) {
      std::cerr << "Weird: landmark " << ld.id() << " is not in landmarks.\n";
      continue;
    }
    // Remove the landmark if it is observed only by this frame and another
    // frame.
    Landmark landmark = *landmark_ptr->second.get();
    // TODO: Check exists.
    landmark.keyframe_to_feature.erase(frame_id);
    landmark.keyframe_to_feature_id.erase(frame_id);

    if (landmark.keyframe_to_feature.size() <= 2) {
      // Delete references from keyframe to this landmark.
      for (auto &frame_to_feature : landmark.keyframe_to_feature) {
        auto ptr = keyframes_.find(frame_to_feature.first);
        if (ptr == keyframes_.end()) {
          std::cerr << "Error\n";
          continue;
        }
        ptr->second
            ->features[landmark.keyframe_to_feature_id[frame_to_feature.first]]
            .landmark_id = -1;
      }
      // Delete landmark.
      landmarks_.erase(landmark_ptr);
    }
  }
  // Remove this keyframe.
  keyframes_.erase(frame_ptr);
  std::cout << "Remove keyframe " << frame_id.id() << std::endl;
  if (keyframes_.empty()) last_keyframe_ = nullptr;

  landmarks_lock.unlock();
  keyframe_lock.unlock();
  return true;
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

void RemoveUnmatchedFeatures(Keyframe &frame) {
  std::cout << "Feature size reduced from " << frame.features.size();
  auto feature_ptr = frame.features.begin();
  while (feature_ptr != frame.features.end()) {
    if (feature_ptr->second.landmark_id == -1)
      feature_ptr = frame.features.erase(feature_ptr);
    else
      ++feature_ptr;
  }
  std::cout << " to " << frame.features.size() << std::endl;
}

// TODO: How to use it in FeatureTracker to evaluate tracker?
bool ProcessMatchesAndAddToLandmarks(Keyframe *pre_frame, Keyframe *cur_frame,
                                     const std::vector<cv::DMatch> &matches,
                                     Landmarks &landmarks) {
  // TODO: Add test.
  cur_frame->pre_frame_id = pre_frame->frame_id;
  pre_frame->next_frame_id = cur_frame->frame_id;
  for (const auto &match : matches) {
    cur_frame->match_to_pre_frame[match.trainIdx] = match.queryIdx;
    // TODO: How can landmark_id compared to int value? Default constructor!
    if (pre_frame->features[match.queryIdx].landmark_id == -1) {
      // New landmark.
      std::unique_ptr<Landmark> new_landmark =
          std::unique_ptr<Landmark>(new Landmark());
      new_landmark->AddMeasurementInKeyframe(
          pre_frame->frame_id, match.queryIdx,
          pre_frame->features[match.queryIdx].measurement);
      new_landmark->AddMeasurementInKeyframe(
          cur_frame->frame_id, match.trainIdx,
          cur_frame->features[match.trainIdx].measurement);
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
          cur_frame->frame_id, match.trainIdx,
          cur_frame->features[match.trainIdx].measurement);
    }
  }
  return true;
}

void RemoveShortTrackLengthLandmark(LandmarkId landmark_id,
                                    Landmarks &landmarks,
                                    Keyframes &keyframes) {
  // TODO
}

void RemoveShortTracks(Landmarks &landmarks, Keyframes &keyframes,
                       KeyframeId &cur_keyframe_id) {
  // TODO
}

bool CopyDataForInitializer(
    const Landmarks &landmarks, const Keyframes &keyframes,
    std::vector<KeyframeId> &frame_ids,
    std::vector<std::vector<cv::Vec2d> > &feature_vectors) {
  const int num_frames_needed_for_init = 2;

  feature_vectors.resize(num_frames_needed_for_init);
  frame_ids.resize(num_frames_needed_for_init);
  int frame_count = 0;
  /*
   * Map KeyframeId to the feature_vectors.
   *
   * So for example, there are two frames with id 111 and 222:
   * feature_ids = { 111, 222 }
   * keyframe_id_to_feature_vector_id = { 111 -> 0, 222 -> 1 }
   * feature_vectors = { measurements in 111, measurement in 222 }
   *
   */
  std::unordered_map<KeyframeId, int> keyframe_id_to_feature_vector_id;
  for (const auto &keyframe_ptr : keyframes) {
    frame_ids[frame_count] = keyframe_ptr.second->frame_id;
    keyframe_id_to_feature_vector_id[keyframe_ptr.second->frame_id] =
        frame_count;
    frame_count++;

    if (frame_count == num_frames_needed_for_init) break;
  }

  // TODO: Doesn't need to go through all landmarks.
  for (const auto &landmark_ptr : landmarks) {
    const Landmark &landmark = *(landmark_ptr.second);
    bool is_landmark_visible_to_all = true;
    // Go through the landmark see if it's visible to all keyframes to be
    // processed.
    // TODO: Only add landmarks that are visible to some of the frames.
    // Now only add landmarks that are visible to all frames.
    for (auto &frame_id : frame_ids) {
      if (landmark.keyframe_to_feature.find(frame_id) ==
          landmark.keyframe_to_feature.end()) {
        is_landmark_visible_to_all = false;
        break;
      }
    }

    // Second pass: add a new feature for initialization.
    if (is_landmark_visible_to_all) {
      for (auto &frame_id : frame_ids) {
        const auto &measurement_ptr =
            landmark.keyframe_to_feature.find(frame_id);
        feature_vectors[keyframe_id_to_feature_vector_id[frame_id]].push_back(
            cv::Vec2d(measurement_ptr->second.x, measurement_ptr->second.y));
      }
    }
  }

  return true;
}

}  // vio
