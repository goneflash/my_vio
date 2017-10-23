#ifndef FEATURE_TRACKS_HPP_
#define FEATURE_TRACKS_HPP_

#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>

#include "mapdata_types.hpp"

namespace vio {

// Position of the feature in image.
struct FeatureMeasurement {
  FeatureMeasurement() : x(-1), y(-1) {}
  FeatureMeasurement(float _x, float _y) : x(_x), y(_y) {}
  float x, y;
};

struct Feature {
  FeatureMeasurement measurement;
  // TODO: Change to Eigen.
  // cv::Mat descriptor;
};

class Landmark {
 public:
  Landmark() : landmark_id(CreateNewId<LandmarkId>()) {}

  // TODO: Should not add duplicated features.
  // TODO: Use Assert of glog.
  bool AddMeasurementInKeyframe(KeyframeId frame_id,
                                FeatureMeasurement measurement) {
    if (keyframe_to_feature.find(frame_id) != keyframe_to_feature.end())
      return false;
    keyframe_to_feature[frame_id] = measurement;
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

  typedef std::unordered_map<KeyframeId, FeatureMeasurement>
      KeyframeIdToFeatureMeasurement;

  KeyframeIdToFeatureMeasurement keyframe_to_feature;

  LandmarkId landmark_id;

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
