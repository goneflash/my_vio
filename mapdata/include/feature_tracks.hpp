#ifndef FEATURE_TRACKS_HPP_
#define FEATURE_TRACKS_HPP_

#include <vector>
#include <unordered_map>

#include "mapdata_types.hpp"

namespace vio {

struct FeatureMeasurement {
  Eigen::Vector2f keypoint;
};

class FeatureTrack {
 public:
  FeatureTrack() : landmark_id(-1) {}

  LandmarkId landmark_id;
  std::unordered_map<KeyFrameId, FeatureMeasurement> measurements;
};

class FeatureTracks {
 public:
  std::vector<FeatureTrack> active_tracks;

  std::unordered_map<LandmarkId, FeatureTrack> past_tracks;
};

}  // vio

#endif  // FEATURE_TRACKS_HPP_
