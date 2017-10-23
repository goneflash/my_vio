#ifndef FEATURE_TRACKS_HPP_
#define FEATURE_TRACKS_HPP_

#include <memory>
#include <vector>
#include <unordered_map>

#include "mapdata_types.hpp"

namespace vio {

class Landmark {
 public:
  Landmark() : landmark_id_(CreateNewId<LandmarkId>()) {}

  LandmarkId landmark_id() { return landmark_id_; }

  // TODO: Should not add duplicated features.
  // TODO: Use Assert of glog.
  bool AddMeasurementInKeyframe(KeyframeId frame_id,
                                FeatureMeasurement measurement) {
    if (keyframe_to_feature_.find(frame_id) != keyframe_to_feature_.end())
      return false;
    keyframe_to_feature_[frame_id] = measurement;
    return true;
  }

  // TODO: Add test.
  bool GetMeasurementInKeyframe(KeyframeId frame_id,
                                FeatureMeasurement &measurement) {
    const auto ptr = keyframe_to_feature_.find(frame_id);
    if (ptr == keyframe_to_feature_.end()) return false;
    measurement = ptr->second;
    return true;
  }

 private:
  typedef std::unordered_map<KeyframeId, FeatureMeasurement>
      KeyframeIdToFeatureMeasurement;

  KeyframeIdToFeatureMeasurement keyframe_to_feature_;

  LandmarkId landmark_id_;
};

/*
class Landmarks {
 public:
  std::unordered_map<LandmarkId, Landmark> ;
};
*/

}  // vio

#endif  // FEATURE_TRACKS_HPP_
