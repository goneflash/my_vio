#include "feature_tracks.hpp"

namespace vio {

void GetLandmarkStats(const Landmarks &landmarks, LandmarkStats &stats) {
  stats.landmark_length.clear();
  for (const auto &landmark_ptr : landmarks) {
    const Landmark &landmark = *landmark_ptr.second;
    const size_t len = landmark.keyframe_to_feature.size();
    if (stats.landmark_length.find(len) == stats.landmark_length.end())
      stats.landmark_length[len] = 0;
    stats.landmark_length[len]++;
  }
}

}  // vio
