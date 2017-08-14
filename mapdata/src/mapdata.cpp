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
  return true;
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

  std::unordered_map<int, int> new_frame_feature_to_uninited_landmark;
  for (int i = 0; i < matches.size(); ++i) {
    // Find existing landmark
    auto ld_id_ptr = feature_to_landmark_[l_index].find(matches[i].queryIdx);
    // If the feature is not a landmark yet
    if (ld_id_ptr == feature_to_landmark_[l_index].end()) {
      // Find if this feature extends an uninited feature tracks
      auto uninited_id_ptr = last_frame_feature_to_uninited_landmark_.find(matches[i].queryIdx);
      if (uninited_id_ptr == last_frame_feature_to_uninited_landmark_.end()) { 
        // Not found, Add a new landmark
        const int num_uninited_ld = uninited_landmark_to_feature_.size();
        uninited_landmark_to_feature_.resize(num_uninited_ld + 1);
        uninited_landmark_to_feature_.back()[n_index] = matches[i].trainIdx;
        uninited_landmark_to_feature_.back()[l_index] = matches[i].queryIdx;
        new_frame_feature_to_uninited_landmark[matches[i].trainIdx] = num_uninited_ld;
      } else {
        // Found an uninited feature
        int uninited_id = uninited_id_ptr->second;
        uninited_landmark_to_feature_[uninited_id][n_index] = matches[i].trainIdx;
        new_frame_feature_to_uninited_landmark[matches[i].trainIdx] = uninited_id;
      }
    } else {
      int landmark_id = ld_id_ptr->second;
      // Link a feature to a landmark
      feature_to_landmark_[n_index][matches[i].trainIdx] = landmark_id;
      // Link a landmark to a feature in the new frame
      landmark_to_feature_[landmark_id][n_index] = matches[i].trainIdx;
    }
  }
  last_frame_feature_to_uninited_landmark_ = std::move(new_frame_feature_to_uninited_landmark);

  match_edges_.back().matches = std::move(matches);
  if (map_state_ == WAIT_FOR_SECONDFRAME) map_state_ = WAIT_FOR_INIT;

  return true;
}

bool Mapdata::PrintStats() {
  std::cout << "\nMap stats:\n"
            << "Keyframes: " << keyframes_.size() << std::endl
            << "Landmarks: " << landmarks_.size() << "\n\n";
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
