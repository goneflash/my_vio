#ifndef FEATURE_TRACKS_HPP_
#define FEATURE_TRACKS_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>

#include "mapdata_types.hpp"

// TODO: Rename file to landmark.
namespace vio {

typedef int FeatureId;

// Position of the feature in image.
struct FeatureMeasurement {
  FeatureMeasurement() : x(-1), y(-1) {}
  FeatureMeasurement(float _x, float _y) : x(_x), y(_y) {}
  float x, y;
};

class Landmark {
 public:
  typedef Eigen::Vector3d Position;

  Landmark()
      : landmark_id(CreateNewId<LandmarkId>()),
        current_track_length(0),
        inited_(false) {}

  // TODO: Should not add duplicated features.
  // TODO: Use Assert of glog.
  bool AddMeasurementInKeyframe(KeyframeId frame_id, FeatureId feature_id,
                                FeatureMeasurement measurement) {
    if (keyframe_to_feature.find(frame_id) != keyframe_to_feature.end())
      return false;
    keyframe_to_feature[frame_id] = measurement;
    keyframe_to_feature_id[frame_id] = feature_id;
    return true;
  }

  // TODO: Add test.
  bool GetMeasurementInKeyframe(KeyframeId frame_id,
                                FeatureMeasurement &measurement) {
    const auto ptr = keyframe_to_feature.find(frame_id);
    if (ptr == keyframe_to_feature.end()) return false;
    measurement = ptr->second;
    return true;
  }

  bool GetFeatureIdInKeyframe(KeyframeId frame_id, FeatureId &feature_id) {
    const auto ptr = keyframe_to_feature_id.find(frame_id);
    if (ptr == keyframe_to_feature_id.end()) return false;
    feature_id = ptr->second;
    return true;
  }

  typedef std::unordered_map<KeyframeId, FeatureMeasurement>
      KeyframeIdToFeatureMeasurement;
  typedef std::unordered_map<KeyframeId, FeatureId> KeyframeIdToFeatureId;

  // TODO: Combine feature and feature id to an obj, or just need feature id.
  KeyframeIdToFeatureMeasurement keyframe_to_feature;
  KeyframeIdToFeatureId keyframe_to_feature_id;

  LandmarkId landmark_id;

  int current_track_length;

  bool inited() { return inited_; }
  bool inited_;
  // In world frame.
  Position position;

 private:
};

typedef std::unordered_map<LandmarkId, std::unique_ptr<Landmark>> Landmarks;

struct LandmarkStats {
  std::unordered_map<size_t, int> landmark_length;
  void Print() {
    std::cout << "Track length stats:\n";
    for (const auto &len : landmark_length) {
      std::cout << "Length " << len.first << " : " << len.second << std::endl;
    }
  }
};

void GetLandmarkStats(const Landmarks &landmarks, LandmarkStats &stats);

}  // vio

#endif  // FEATURE_TRACKS_HPP_
