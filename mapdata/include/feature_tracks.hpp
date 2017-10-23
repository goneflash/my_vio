#ifndef FEATURE_TRACKS_HPP_
#define FEATURE_TRACKS_HPP_

#include <memory>
#include <vector>
#include <unordered_map>

#include "mapdata_types.hpp"

namespace vio {


class FeatureTrack {
 public:
  FeatureTrack() : landmark_id(-1) {}

  LandmarkId landmark_id;
  std::unordered_map<KeyFrameId, FeatureMeasurement> measurements;
};

/*
class Landmark {
  public:

  
};
*/

class Landmarks {
 public:
  std::unordered_set<LandmarkId> active_landmarks;
  std::unordered_map<LandmarkId, FeatureTrack> tracks;
};

}  // vio

#endif  // FEATURE_TRACKS_HPP_
