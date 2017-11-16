#include "visual_inertial_odometry.hpp"

#include "multiview.hpp"
#include "timer.hpp"

namespace vio {

VisualInertialOdometry::VisualInertialOdometry(CameraModelPtr camera)
    : camera_(std::move(camera)),
      vio_status_(UNINITED),
      // TODO: Remove and use keyframes_.end()
      last_keyframe_(nullptr),
      running_process_buffer_thread_(false),
      running_initializer_thread_(false),
      num_skipped_frames_(0) {
  // Setup Feature tracker.
  InitializeFeatureTracker();
  InitializeVIOInitializer();
  pnp_estimator_ = PnPEstimator::CreatePnPEstimator(ITERATIVE);
  cv::namedWindow("tracking", cv::WINDOW_AUTOSIZE);

  track_length_to_landmark_.resize(3);
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

bool VisualInertialOdometry::ProcessNewImage(cv::Mat &img) {
  return data_buffer_.AddImageData(img);
}

void VisualInertialOdometry::ProcessDataInBuffer() {
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  Timer timer;

  for (;;) {
    if (!running_process_buffer_thread_) break;

    cv::Mat new_image;
    if (data_buffer_.GetImageDataOrEndOfBuffer(new_image)) break;
    // Return false means the image is skipped.
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
        std::vector<std::vector<cv::Vec2d>> feature_vectors;
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
      keyframe_lock.lock();

      const auto &ptr = keyframes_.find(last_keyframe_->pre_frame_id);
      if (ptr == keyframes_.end()) {
        std::cerr << "Error: No previous keyframes exist.\n";
        break;
      }
      Keyframe &pre_keyframe = *(ptr->second);
      // Initialization might not propagate to here yet.
      if (!pre_keyframe.inited_pose()) {
        keyframe_lock.unlock();
        std::cout << "Added an uninited keyframe.\n";
        continue;
      }
      keyframe_lock.unlock();

      timer.Start();

      // TODO: If don't lock, keyframes might change.
      if (!InitializePoseForNewKeyframe(pre_keyframe, *last_keyframe_)) {
        std::cerr << "Error: Can't initialize new keyframe. Need to restart.\n";
        std::cerr << "Now quiting...\n";
        break;
      }

      if (!TriangulteLandmarksInNewKeyframes(pre_keyframe, *last_keyframe_)) {
        std::cerr << "Error: Can't triangulate landmarks for new keyframe.\n";
        std::cerr << "Now quiting...\n";
        break;
      }

      timer.Stop();
      std::cout << "Initialize new keyframe and triagulate landmarks used "
                << timer.GetInMs() << "ms.\n";

      // Estmiate the pose of current frame.
      // TODO: Wait until all keyframes are pose_inited.
    }
  }
  data_buffer_.CloseBuffer();
}

bool VisualInertialOdometry::AddNewKeyframeFromImage(const cv::Mat &new_image) {
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  Timer timer;

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
  if (matches.size() < 10) {
    std::cout << "Warning: Lost tracking. Restarting...";
    // TODO: Should stop.
    return false;
  } else {
    // Add this frame as a new keyframe.
    std::unique_ptr<Keyframe> new_keyframe =
        std::unique_ptr<Keyframe>(new Keyframe(std::move(frame_cur)));

    keyframe_lock.lock();
    if (ShouldSkipThisFrame(last_keyframe_, new_keyframe.get(), matches) &&
        num_skipped_frames_ < 8) {
      num_skipped_frames_++;
      keyframe_lock.unlock();
      return false;
    }
    keyframe_lock.unlock();

    num_skipped_frames_ = 0;

    timer.Start();

    // Add tracks.
    std::lock(landmarks_lock, keyframe_lock);
    ProcessMatchesAndAddToLandmarks(last_keyframe_, new_keyframe.get(), matches,
                                    track_length_to_landmark_, landmarks_);
    RemoveUnmatchedFeatures(*last_keyframe_);
    RemoveShortTracksNotVisibleToCurrentKeyframe(last_keyframe_->frame_id);
    landmarks_lock.unlock();
    keyframe_lock.unlock();

    timer.Stop();
    std::cout << "Process and clean matches used " << timer.GetInMs()
              << "ms.\n";

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

void VisualInertialOdometry::RemoveShortTracksNotVisibleToCurrentKeyframe(
    const KeyframeId &cur_keyframe_id) {
  const int min_track_length = 4;
  // TODO: Could use vector or unordered_map to store index so that don't need
  // to iterate through all landmarks.
  const int initial_num_landmarks = landmarks_.size();
  int count = 0;
  for (int len = 2; len < min_track_length; ++len) {
    if (len + 1 > track_length_to_landmark_.size()) break;

    auto landmark_id_ptr = track_length_to_landmark_[len].begin();
    while (landmark_id_ptr != track_length_to_landmark_[len].end()) {
      auto &landmark_id = *landmark_id_ptr;
      Landmark &landmark = *landmarks_[landmark_id];
      // If not visible to current keyframe then delete this landmark.
      if (landmark.keyframe_to_feature.find(cur_keyframe_id) ==
          landmark.keyframe_to_feature.end()) {
        // Remove all reference from keyframes.
        for (auto &frame_to_feature : landmark.keyframe_to_feature) {
          auto ptr = keyframes_.find(frame_to_feature.first);
          if (ptr == keyframes_.end()) {
            std::cerr << "Error\n";
            continue;
          }
          ptr->second->features
              [landmark.keyframe_to_feature_id[frame_to_feature.first]]
                  .landmark_id = -1;
        }
        landmarks_.erase(landmark_id);
        landmark_id_ptr = track_length_to_landmark_[len].erase(landmark_id_ptr);
        count++;
      } else {
        landmark_id_ptr++;
      }
    }
  }
  std::cout << "Reduced landmarks from " << initial_num_landmarks << " to "
            << landmarks_.size() << std::endl;
}

void VisualInertialOdometry::RunInitializer(
    const std::vector<KeyframeId> &frame_ids,
    const std::vector<std::vector<cv::Vec2d>> &feature_vectors) {
  std::vector<cv::Point3f> points3d;
  std::vector<bool> points3d_mask;
  std::vector<cv::Mat> Rs_est, ts_est;

  Timer timer;
  timer.Start();
  // TODO: Change this.
  // cv::Matx33d K_ = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);
  const cv::Mat &K = camera_->camera_matrix();
  const bool success = map_initializer_->Initialize(
      feature_vectors, K, points3d, points3d_mask, Rs_est, ts_est);

  timer.Stop();
  std::cout << "Initialization used " << timer.GetInMs() << "ms.\n";

  if (!success) {
    std::cerr << "Warning: Initialization failed.\n\n";
    // TODO: Clear all keyframes and landmarks.
    // Probably want to only remove the keyframes that failed initialization.
    // There are other cases, e.g. loop closure, lost tracking.
    timer.Start();

    for (auto &frame_id : frame_ids) {
      if (!RemoveKeyframe(frame_id)) {
        std::cout << "Fatal Error: Failed to remove keyframe.\n";
      }
      std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_);
      std::cout << "Removed keyframes. Now " << keyframes_.size() << " left.\n";
      // TODO: For now only delete the first keyframe.
      break;
    }

    timer.Stop();
    std::cout << "Removing keyframes used " << timer.GetInMs() << "ms.\n";

  } else {
    std::cerr << "Initialization Success.\n\n";

    timer.Start();
    CopyInitializedFramesAndLandmarksData(frame_ids, Rs_est, ts_est);
    vio_status_ = INITED;

    timer.Stop();
    std::cout << "Copy initialization data used " << timer.GetInMs() << "ms.\n";

    // TODO: Should do it here?
    // Propagate to all keyframes.
    auto keyframe_ptr = keyframes_.begin();
    while (keyframe_ptr != keyframes_.end()) {
      // TODO: Now assume if a keyframe is initialized, it will at the same
      // triangulate new landmarks.
      if (keyframe_ptr->second->inited_pose()) {
        keyframe_ptr++;
        continue;
      }

      std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_,
                                                 std::defer_lock);
      keyframe_lock.lock();
      Keyframe &new_keyframe = *(keyframe_ptr->second);
      if (new_keyframe.pre_frame_id == -1) break;
      const auto &ptr = keyframes_.find(new_keyframe.pre_frame_id);
      if (ptr == keyframes_.end()) break;
      Keyframe &pre_keyframe = *(ptr->second);
      if (!pre_keyframe.inited_pose()) break;
      keyframe_lock.unlock();

      timer.Start();

      // TODO: If don't lock, keyframes might change.
      if (!InitializePoseForNewKeyframe(pre_keyframe, new_keyframe)) {
        std::cerr << "Error: Can't propagate initialization. Need to redo.\n";
        break;
      }

      if (!TriangulteLandmarksInNewKeyframes(pre_keyframe, new_keyframe)) {
        std::cerr << "Error: Can't triangulate new landmarks.\n";
        break;
      }

      timer.Stop();
      std::cout << "Propagate to a new keyframe and triagulate landmarks used "
                << timer.GetInMs() << "ms.\n";

      keyframe_ptr++;
    }
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
    std::cout << "Set pose for keyframe: " << frame_ids[i].id() << std::endl;
  }
  keyframe_lock.unlock();

  // TODO: For now, only support two frames.
  if (frame_ids.size() != 2) {
    std::cout << "Nononono...only 2 frames.\n";
    return;
  }

  // TODO: Change this.
  // cv::Matx33d K = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);
  const cv::Mat &K = camera_->camera_matrix();

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
    // TODO: How to triangulate a point from 3+ frames.
    cv::Point3f point_3d;
    TriangulateDLT(kp[0], kp[1], P[0], P[1], point_3d);
    if (IsGoodTriangulatedPoint(kp[0], kp[1], R[0], t[0], R[1], t[1], P[0],
                                P[1], point_3d)) {
      landmark_ptr.second->position[0] = point_3d.x;
      landmark_ptr.second->position[1] = point_3d.y;
      landmark_ptr.second->position[2] = point_3d.z;
      // TODO: private.
      landmark_ptr.second->inited_ = true;
      good_count++;
    }
  }
  landmarks_lock.unlock();
  keyframe_lock.unlock();
  std::cout << "Tested " << tested_count << " landmarks.\n";
  std::cout << "Triangulated " << good_count << " landmarks.\n";
}

// TODO: Consider move out of class.
bool VisualInertialOdometry::InitializePoseForNewKeyframe(Keyframe &pre_frame,
                                                          Keyframe &new_frame) {
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);

  // TODO: How to increase the lock granularity here.
  // previous keyframe.

  // TODO: Also try to five point method using 2D-2D matches.

  // Gather data for estimating the pose.
  std::vector<cv::Point3f> points3d;
  std::vector<cv::Point2f> points2d;
  // TODO
  std::lock(keyframe_lock, landmarks_lock);
  for (const auto &match : new_frame.match_to_pre_frame) {
    const auto &ld = pre_frame.features[match.second].landmark_id;
    if (ld == -1 || !landmarks_[ld]->inited()) continue;
    points3d.push_back(cv::Point3f(landmarks_[ld]->position[0],
                                   landmarks_[ld]->position[1],
                                   landmarks_[ld]->position[2]));
    points2d.push_back(
        cv::Point2f(new_frame.features[match.first].measurement.x,
                    new_frame.features[match.first].measurement.y));
  }

  landmarks_lock.unlock();
  keyframe_lock.unlock();

  if (points3d.size() < 10) {
    std::cout << "Not enough pairs for PnP.\n";
    return false;
  }

  // TODO: Change this.
  // cv::Matx33d K = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);
  const cv::Mat &K = camera_->camera_matrix();

  std::vector<bool> inliers;
  cv::Mat R;
  cv::Mat t;
  if (!pnp_estimator_->EstimatePose(points2d, points3d, cv::Mat(K), inliers, R,
                                    t)) {
    std::cerr << "Error: PnP estimation.\n";
    return false;
  }

  keyframe_lock.lock();
  new_frame.SetPose(R, t);
  keyframe_lock.unlock();
  return true;
}

bool VisualInertialOdometry::TriangulteLandmarksInNewKeyframes(
    Keyframe &pre_frame, Keyframe &new_frame) {
  std::unique_lock<std::mutex> landmarks_lock(landmarks_mutex_,
                                              std::defer_lock);
  std::unique_lock<std::mutex> keyframe_lock(keyframes_mutex_, std::defer_lock);

  // TODO: Change this.
  // cv::Matx33d K = cv::Matx33d(650, 0, 320, 0, 650, 240, 0, 0, 1);
  const cv::Mat &K = camera_->camera_matrix();

  std::lock(landmarks_lock, keyframe_lock);
  if (!new_frame.inited_pose()) return false;

  int good_count = 0, tested_count = 0;
  for (const auto &match : new_frame.match_to_pre_frame) {
    const auto &ld = pre_frame.features[match.second].landmark_id;
    // TODO: Don't have to triangulate points until > 3 frames see it?
    // Already triangulated?
    if (ld == -1 || landmarks_[ld]->inited()) continue;

    Landmark &landmark = *landmarks_[ld];
    std::vector<cv::Vec2d> kp;
    std::vector<cv::Mat> P, R, t;

    // Add measurement for all frames to triangulate.
    // TODO: Now only supports two views.
    for (auto &ptr : landmark.keyframe_to_feature) {
      auto keyframe_ptr = keyframes_.find(ptr.first);
      if (keyframe_ptr == keyframes_.end()) {
        std::cerr << "Weird: keyframe " << ptr.first.id()
                  << " is not in keyframes.\n";
        continue;
      }

      Keyframe &keyframe = *(keyframe_ptr->second);
      // TODO: there might be new keyframes already added that also see this
      // landmark.
      if (!keyframe.inited_pose()) continue;

      kp.push_back(cv::Vec2d(ptr.second.x, ptr.second.y));
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
    // TODO: How to triangulate a point from 3+ frames.
    cv::Point3f point_3d;
    TriangulateDLT(kp[0], kp[1], P[0], P[1], point_3d);
    if (IsGoodTriangulatedPoint(kp[0], kp[1], R[0], t[0], R[1], t[1], P[0],
                                P[1], point_3d)) {
      landmark.position[0] = point_3d.x;
      landmark.position[1] = point_3d.y;
      landmark.position[2] = point_3d.z;
      // TODO: private.
      landmark.inited_ = true;
      good_count++;
    }
  }
  landmarks_lock.unlock();
  keyframe_lock.unlock();
  std::cout << "Triangulated " << good_count << " / " << tested_count
            << " new landmarks.\n";
  return true;
}

bool VisualInertialOdometry::RemoveKeyframe(KeyframeId frame_id) {
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
    LandmarkId &ld = feature.second.landmark_id;
    if (ld == -1) continue;
    auto landmark_ptr = landmarks_.find(ld);
    if (landmark_ptr == landmarks_.end()) {
      std::cerr << "Weird: landmark " << ld.id() << " is not in landmarks.\n";
      continue;
    }
    // Remove the landmark if it is observed only by this frame and another
    // frame.
    Landmark &landmark = *landmark_ptr->second;
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
      track_length_to_landmark_[landmark.current_track_length].erase(
          landmark.landmark_id);
      landmarks_.erase(landmark_ptr);
    }
  }

  // If it's a middle keyframe, must connect previous frame and next frame.
  if (keyframe->next_frame_id.valid() && keyframe->pre_frame_id.valid()) {
    auto next_keyframe_ptr = keyframes_.find(keyframe->next_frame_id);
    auto prev_keyframe_ptr = keyframes_.find(keyframe->pre_frame_id);
    if (next_keyframe_ptr == keyframes_.end()) {
      std::cerr << "Error: Next keyframe doesn't exist but specified.\n";
      return false;
    }
    if (prev_keyframe_ptr == keyframes_.end()) {
      std::cerr << "Error: Previous keyframe doesn't exist but specified.\n";
      return false;
    }
    auto next_keyframe = next_keyframe_ptr->second.get();
    auto prev_keyframe = prev_keyframe_ptr->second.get();
    prev_keyframe->next_frame_id = next_keyframe->frame_id;
    next_keyframe->pre_frame_id = prev_keyframe->frame_id;
    // Recompute the vision constraint between the keyframes.
    // TODO: May be redo the tracking?
  }

  // TODO: If it's the only keyframe.
  if (keyframes_.size() == 1) {
    keyframes_.clear();
    return true;
  }

  // Remove this keyframe.
  keyframes_.erase(frame_id);
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

  // Add frames.
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

  // Add landmarks.
  for (const auto &id_to_landmark : landmarks_) {
    const auto &landmark = id_to_landmark.second;
    if (landmark->inited()) {
      scene.landmarks.push_back(*landmark);
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

bool ShouldSkipThisFrame(Keyframe *frame0, Keyframe *frame1,
                         const std::vector<cv::DMatch> &matches) {
  // Estimate motion, skip if it's static.
  int excceed_thresh_count = 0;
  for (const auto &match : matches) {
    if (feature_dist(frame0->features[match.queryIdx].measurement,
                     frame1->features[match.trainIdx].measurement) > 4.0f)
      excceed_thresh_count++;
  }
  // TODO: Find better criterion.
  // if (excceed_thresh_count < 30 && matches.size() > 300) {
  if ((double)excceed_thresh_count / (double)matches.size() < 0.4) {
    return true;
  }
  return false;
}

// TODO: How to use it in FeatureTracker to evaluate tracker?
bool ProcessMatchesAndAddToLandmarks(
    Keyframe *pre_frame, Keyframe *cur_frame,
    const std::vector<cv::DMatch> &matches,
    std::vector<std::unordered_set<LandmarkId>> &track_length_to_landmark,
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
      // Add to track length index.
      track_length_to_landmark[2].insert(new_landmark->landmark_id);
      new_landmark->current_track_length = 2;
      // Add landmark.
      landmarks[new_landmark->landmark_id] = std::move(new_landmark);
    } else {
      // Handle existing landmark.
      LandmarkId landmark_id = pre_frame->features[match.queryIdx].landmark_id;
      cur_frame->features[match.trainIdx].landmark_id = landmark_id;
      Landmark &landmark = *landmarks[landmark_id];
      landmark.AddMeasurementInKeyframe(
          cur_frame->frame_id, match.trainIdx,
          cur_frame->features[match.trainIdx].measurement);
      // Change the track length index.
      track_length_to_landmark[landmark.current_track_length].erase(
          landmark_id);
      landmark.current_track_length++;
      if (track_length_to_landmark.size() < landmark.current_track_length + 1) {
        track_length_to_landmark.resize(landmark.current_track_length + 1);
        track_length_to_landmark[landmark.current_track_length].clear();
      }
      track_length_to_landmark[landmark.current_track_length].insert(
          landmark_id);
    }
  }
  return true;
}

bool CopyDataForInitializer(
    const Landmarks &landmarks, const Keyframes &keyframes,
    std::vector<KeyframeId> &frame_ids,
    std::vector<std::vector<cv::Vec2d>> &feature_vectors) {
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
