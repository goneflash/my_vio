#include "mapdata.hpp"

#include <iostream>

namespace vio {

Mapdata::Mapdata() : map_state_(WAIT_FOR_FIRSTFRAME), min_pnp_matches_(10) {}

bool Mapdata::AddFirstKeyframe(std::unique_ptr<Keyframe> frame) {
  if (map_state_ != WAIT_FOR_FIRSTFRAME) {
    std::cerr << "Error: First Keyframe already exists. Try reset map.\n";
    return false;
  }
  keyframe_id_to_index_[frame->frame_id()] = 0;
  keyframes_.push_back(std::move(frame));
  // Set pose of first frame
  cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat t = cv::Mat::ones(3, 1, CV_64F);
  keyframes_[0]->set_pose(R, t);

  map_state_ = WAIT_FOR_SECONDFRAME;
}

bool Mapdata::AddNewKeyframeMatchToLastKeyframe(
    std::unique_ptr<Keyframe> frame, std::vector<cv::DMatch> &matches) {
  if (map_state_ == WAIT_FOR_FIRSTFRAME) {
    std::cerr << "Error: Missing first frame.\n";
    return false;
  }
  keyframe_id_to_index_[frame->frame_id()] = keyframes_.size();
  keyframes_.push_back(std::move(frame));

  // -------------- Add new match edge
  match_edges_.resize(match_edges_.size() + 1);
  match_edges_.back().first_frame_index = keyframes_.size() - 2;
  match_edges_.back().second_frame_index = keyframes_.size() - 1;
  const int l_index =
      match_edges_.back().first_frame_index;  // last frame index
  const int n_index =
      match_edges_.back().second_frame_index;  // new frame index

  feature_to_landmark_.resize(n_index + 1);
  for (int i = 0; i < matches.size(); ++i) {
    // Find existing landmark
    auto ld_id_ptr = feature_to_landmark_[l_index].find(matches[i].queryIdx);
    // If the feature is not a landmark yet
    if (ld_id_ptr == feature_to_landmark_[l_index].end()) {
      // Add a new landmark
      const int num_uninited_ld = uninited_landmark_to_feature_.size();
      uninited_landmark_to_feature_.resize(num_uninited_ld + 1);
      uninited_landmark_to_feature_.back()[n_index] = matches[i].trainIdx;
      uninited_landmark_to_feature_.back()[l_index] = matches[i].queryIdx;
    } else {
      int landmark_id = ld_id_ptr->second;
      // Link a feature to a landmark
      feature_to_landmark_[n_index][matches[i].trainIdx] = landmark_id;
      // Link a landmark to a feature in the new frame
      landmark_to_feature_[landmark_id][n_index] = matches[i].trainIdx;
    }
  }

  match_edges_.back().matches = std::move(matches);
  if (map_state_ == WAIT_FOR_SECONDFRAME) map_state_ = WAIT_FOR_INIT;

  return true;
}

bool Mapdata::DropLastKeyframe() {
  if (keyframes_.size() < 2) {
    std::cout << "Warning: Trying to drop first frame.\n";
    return false;
  }

  const FeatureMatchEdge &match_edge = match_edges_.back();
  const int ll_index =
      match_edges_.back().first_frame_index;  // last frame index
  const int l_index =
      match_edges_.back().second_frame_index;  // new frame index
  for (int i = 0; i < match_edge.matches.size(); ++i) {
    auto ld_id_ptr =
        feature_to_landmark_[l_index].find(match_edge.matches[i].trainIdx);
    if (ld_id_ptr != feature_to_landmark_[l_index].end()) {
      int landmark_id = ld_id_ptr->second;
      landmark_to_feature_[landmark_id].erase(l_index);
      // If the landmark only point to this feature and another feature
      if (landmark_to_feature_[landmark_id].size() < 2) {
      }
    }
  }

  // TODO: This pretty redundant. Change to not add?
  // PruneShortTrackLandmarks();

  feature_to_landmark_.pop_back();
  match_edges_.pop_back();
  keyframes_.pop_back();
  uninited_landmark_to_feature_.clear();

  return true;
}

bool Mapdata::PrepareInitializationData(
    std::vector<std::vector<cv::Vec2d> > &feature_vectors) {
  if (map_state_ != WAIT_FOR_INIT) {
    std::cerr << "Error: Could not initialize.\n";
    return false;
  }
  std::cout << "Prepare initialization data for " << keyframes_.size()
            << " frames.\n";
  feature_vectors.resize(keyframes_.size());

  const int start_frame_id = 0;
  const int end_frame_id = keyframes_.size() - 1;
  const int num_landmark = uninited_landmark_to_feature_.size();

  // Initialize feature vector
  // TODO: Optimize, if the initialization cost much time.
  for (int frame_id = 0; frame_id < keyframes_.size(); ++frame_id) {
    feature_vectors[frame_id].resize(num_landmark, cv::Vec2d(-1, -1));
  }
  for (int ld_id = 0; ld_id < num_landmark; ++ld_id) {
    for (auto &ld_feature_id : uninited_landmark_to_feature_[ld_id]) {
      const cv::KeyPoint &kp = (keyframes_[ld_feature_id.first]
                                    ->image_frame()
                                    .keypoints())[ld_feature_id.second];
      feature_vectors[ld_feature_id.first][ld_id] = cv::Vec2d(kp.pt.x, kp.pt.y);
    }
  }
  return true;
}

bool Mapdata::AddInitialization(const std::vector<cv::Point3f> &points3d,
                                const std::vector<bool> &points3d_mask,
                                const std::vector<cv::Mat> &Rs,
                                const std::vector<cv::Mat> &ts) {
  if (map_state_ != WAIT_FOR_INIT) {
    std::cerr << "Error: Could not add initialization.\n";
    return false;
  }
  landmark_to_feature_.clear();

  if (!AddCoordToUninitedPoints(points3d, points3d_mask)) return false;

  for (int i = 0; i < keyframes_.size(); ++i) {
    keyframes_[i]->set_pose(Rs[i], ts[i]);
    keyframes_[i]->set_pose_inited(true);
  }

  map_state_ = INITIALIZED;
  return true;
}

bool Mapdata::PrepareEstimateLastFramePoseData(
    std::vector<cv::Point3f> &points3d, std::vector<cv::Point2f> &points2d,
    std::vector<int> &points_index) {
  if (map_state_ != INITIALIZED) {
    std::cerr << "Error: Map not initialized yet.\n";
    return false;
  }
  if (keyframes_.back()->pose_inited()) {
    std::cerr << "Error: Last frame already inited pose.\n";
    return false;
  }

  points3d.clear();
  points2d.clear();
  points_index.clear();

  const int l_index = match_edges_.back().first_frame_index;
  const int n_index = match_edges_.back().second_frame_index;
  const std::vector<cv::DMatch> &matches = match_edges_.back().matches;

  // TODO: This is duplicated when add a new keyframe
  for (int i = 0; i < matches.size(); ++i) {
    auto ld_id_ptr = feature_to_landmark_[n_index].find(matches[i].trainIdx);
    // If the feature is not a landmark yet
    if (ld_id_ptr != feature_to_landmark_[n_index].end()) {
      const int ft_id = ld_id_ptr->first;
      const int ld_id = ld_id_ptr->second;
      points_index.push_back(ft_id);
      points3d.push_back(landmarks_[ld_id].position);

      points2d.push_back(
          keyframes_.back()->image_frame().keypoints()[ft_id].pt);
    }
  }

  if (points_index.size() < 8) {
    std::cout << "Not found enough 2d to 3d matches for pnp.\n";
    return false;
  }

  std::cout << "Found " << points_index.size() << " 2d to 3d match for pnp.\n";
  return true;
}

bool Mapdata::PrepareUninitedPointsFromLastTwoFrames(
    std::vector<cv::Vec2d> &kp0, std::vector<cv::Vec2d> &kp1, FramePose &pose0,
    FramePose &pose1) {
  if (!keyframes_.back()->pose_inited() ||
      !keyframes_[keyframes_.size() - 2]->pose_inited()) {
    std::cerr
        << "Error: Need to estimate frame pose before estimating landmarks.\n";
    return false;
  }
  const int num_uninited_landmarks = uninited_landmark_to_feature_.size();
  kp0.clear();
  kp1.clear();
  for (int ld_id = 0; ld_id < num_uninited_landmarks; ++ld_id) {
    for (auto &ld_feature_id : uninited_landmark_to_feature_[ld_id]) {
      const cv::KeyPoint &kp = (keyframes_[ld_feature_id.first]
                                    ->image_frame()
                                    .keypoints())[ld_feature_id.second];
      if (ld_feature_id.first == keyframes_.size() - 1) {
        kp0.push_back(cv::Vec2d(kp.pt.x, kp.pt.y));
      } else if (ld_feature_id.first == keyframes_.size() - 2) {
        kp1.push_back(cv::Vec2d(kp.pt.x, kp.pt.y));
      } else {
        std::cerr << "Error: Uninited landmark point to previous frame.\n";
        return false;
      }
    }
    if (kp0.size() != kp1.size()) {
      std::cerr << "Error: Uninited landmarks not match.\n";
      return false;
    }
  }
  if (kp0.size() != num_uninited_landmarks) {
    std::cerr << "Error: Uninited landmarks not match.\n";
    return false;
  }

  keyframes_.back()->pose().R.copyTo(pose0.R);
  keyframes_.back()->pose().t.copyTo(pose0.t);
  keyframes_[keyframes_.size() - 2]->pose().R.copyTo(pose1.R);
  keyframes_[keyframes_.size() - 2]->pose().t.copyTo(pose1.t);

  return true;
}

bool Mapdata::AddInitedPoints(const std::vector<cv::Point3f> &points3d,
                              const std::vector<bool> &points3d_mask) {
  if (!AddCoordToUninitedPoints(points3d, points3d_mask)) return false;

  if (keyframes_.size() > 2) PruneShortTrackLandmarks();
  return true;
}

bool Mapdata::SetLastFramePose(const cv::Mat &R, const cv::Mat &t) {
  keyframes_.back()->set_pose_inited(true);
  keyframes_.back()->set_pose(R, t);
};

bool Mapdata::PrepareOptimization(std::vector<cv::Mat> &Rs,
                                  std::vector<cv::Mat> &ts,
                                  std::vector<cv::Point3f> &points,
                                  std::vector<int> &obs_camera_idx,
                                  std::vector<int> &obs_point_idx,
                                  std::vector<cv::Vec2d> &obs_feature) {
  const int num_camera = keyframes_.size();
  const int num_points = landmarks_.size();
  Rs.resize(num_camera);
  ts.resize(num_camera);
  points.resize(num_points);
  for (int i = 0; i < keyframes_.size(); ++i) {
    keyframes_[i]->pose().R.copyTo(Rs[i]);
    keyframes_[i]->pose().t.copyTo(ts[i]);
  }

  for (int i = 0; i < num_points; ++i) {
    points[i] = landmarks_[i].position;
  }

  obs_camera_idx.clear();
  obs_point_idx.clear();
  obs_feature.clear();
  for (int i = 0; i < landmarks_.size(); ++i) {
    const int num_obs = landmark_to_feature_[i].size();
    const int obs_start_id = obs_camera_idx.size();

    obs_camera_idx.resize(obs_camera_idx.size() + num_obs);
    obs_point_idx.resize(obs_point_idx.size() + num_obs);
    obs_feature.resize(obs_feature.size() + num_obs);

    int count = 0;
    for (auto &obs : landmark_to_feature_[i]) {
      obs_camera_idx[obs_start_id + count] = obs.first;
      obs_point_idx[obs_start_id + count] = i;

      const cv::KeyPoint &kp =
          keyframes_[obs.first]->image_frame().keypoints()[obs.second];
      obs_feature[obs_start_id + count] = cv::Vec2d(kp.pt.x, kp.pt.y);
      count++;
    }
  }
  return true;
}

bool Mapdata::ApplyOptimization(const std::vector<cv::Mat> &Rs,
                                const std::vector<cv::Mat> &ts,
                                const std::vector<cv::Point3f> &points) {
  const int num_camera = Rs.size();
  const int num_points = points.size();

  for (int i = 0; i < num_camera; ++i) {
    keyframes_[i]->set_pose(Rs[i], ts[i]);
  }

  for (int i = 0; i < points.size(); ++i) {
    landmarks_[i].position = points[i];
    landmarks_[i].optimized = true;
  }
  return true;
}

bool Mapdata::PrintStats() {
  std::cout << "\nMap stats:\n"
            << "Keyframes: " << keyframes_.size() << std::endl
            << "Landmarks: " << landmarks_.size() << "\n\n";
}

bool Mapdata::AddCoordToUninitedPoints(const std::vector<cv::Point3f> &points3d,
                                       const std::vector<bool> &points3d_mask) {
  if (points3d.size() != uninited_landmark_to_feature_.size() ||
      points3d_mask.size() != uninited_landmark_to_feature_.size()) {
    std::cerr << "Error: New points size doesn't match uninited landmarks.\n";
    return false;
  }

  int new_ld_count = 0;
  const int cur_frame_id = keyframes_.size();
  for (int ld_id = 0; ld_id < points3d.size(); ++ld_id) {
    if (points3d_mask[ld_id]) {
      landmark_to_feature_.push_back(
          std::move(uninited_landmark_to_feature_[ld_id]));
      Landmark new_ld;
      new_ld.position = points3d[ld_id];
      new_ld.added_frame_id = cur_frame_id;
      const int landmark_id = landmarks_.size();
      new_ld_count++;
      landmarks_.push_back(new_ld);

      for (auto &feature_in_frame : landmark_to_feature_.back()) {
        const int frame_id = feature_in_frame.first;
        const int feature_id = feature_in_frame.second;
        feature_to_landmark_[frame_id][feature_id] = landmark_id;
      }
    }
  }

  std::cout << "Added inlier triangulated landmarks: " << new_ld_count << " / "
            << points3d.size() << std::endl;
  std::cout << "Number of landmarks: " << landmarks_.size() << std::endl;
  // All uninited points will be delete.
  uninited_landmark_to_feature_.clear();
  return true;
}

bool Mapdata::PruneShortTrackLandmarks() {
  // If previous landmarks only seen by two views then delete.
  // TODO: Could be only used when frame rate is high enough.
  std::vector<Landmark> pruned_landmarks;
  std::vector<std::unordered_map<int, int> > pruned_landmark_to_feature;

  const int last_frame_id = keyframes_.size() - 1;

  // TODO: Options for selecting visibility frame number.
  for (int i = 0; i < landmarks_.size(); ++i) {
    if (landmark_to_feature_[i].size() < 2) continue;
    if (landmark_to_feature_[i].size() == 2) {
      auto ld_to_last_ptr = landmark_to_feature_[i].find(last_frame_id);
      if (ld_to_last_ptr == landmark_to_feature_[i].end()) {
        // Remove feature to landmark mapping
        for (auto &ft_ptr : landmark_to_feature_[i]) {
          feature_to_landmark_[ft_ptr.first].erase(ft_ptr.second);
        }
        continue;
      }
    }
    // Need to remap feature to landmark since their index is changed
    for (auto &ft_ptr : landmark_to_feature_[i]) {
      feature_to_landmark_[ft_ptr.first][ft_ptr.second] =
          pruned_landmarks.size();
    }

    pruned_landmarks.push_back(landmarks_[i]);
    pruned_landmark_to_feature.push_back(std::move(landmark_to_feature_[i]));
  }

  std::cout << "Pruned landmarks from " << landmarks_.size() << " to "
            << pruned_landmarks.size() << std::endl;

  landmarks_ = std::move(pruned_landmarks);
  landmark_to_feature_ = std::move(pruned_landmark_to_feature);

  return true;
}

}  // vio
