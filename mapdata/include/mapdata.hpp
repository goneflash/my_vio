#ifndef VIO_MAP_
#define VIO_MAP_

#include <memory>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "keyframe.hpp"

namespace vio {

struct Landmark {
  cv::Point3f position;
  int added_frame_id;
  bool optimized;
};

struct FeatureMatchEdge {
  std::vector<cv::DMatch> matches;
  int first_frame_index;
  int second_frame_index;
};

class Mapdata {
 public:
  enum MapState {
    WAIT_FOR_FIRSTFRAME = 0,
    // TODO: Might not be only 2 frames.
    WAIT_FOR_SECONDFRAME,
    WAIT_FOR_INIT,
    INITIALIZED,
  };

  Mapdata();

  // TODO: Now assume new keyframe only match to last keyframe
  bool AddFirstKeyframe(std::unique_ptr<Keyframe> frame);
  bool AddNewKeyframeMatchToLastKeyframe(std::unique_ptr<Keyframe> frame,
                                         std::vector<cv::DMatch> &matches);

  bool PrintStats();

  MapState state() { return map_state_; }
  void set_state(MapState state) { map_state_ = state; }

  const Keyframe &GetLastKeyframe() const { return *(keyframes_.back()); }
  int num_frame() const { return keyframes_.size(); }
  const Keyframe &keyframe(int i) const { return *(keyframes_[i]); }

  // The number of landmarks a keyframe can see.
  int num_landmarks_in_frame(int i) const {
    return feature_to_landmark_[i].size();
  }

  int num_landmark() const { return landmarks_.size(); }
  const std::vector<Landmark> &landmarks() { return landmarks_; }
  const Landmark &landmark(int i) { return landmarks_[i]; }

  // Each unordered_map belongs to an uninited landmark.
  // Each map <a, b> in the hashmap records this landmark's feature id
  // is b in keyframe a.
  const std::vector<std::unordered_map<int, int> > &uninited_landmarks() {
    return uninited_landmark_to_feature_; }

 private:
  bool PruneShortTrackLandmarks();

  MapState map_state_;

  std::unordered_map<int, int> keyframe_id_to_index_;
  std::vector<std::unique_ptr<Keyframe> > keyframes_;

  // match_edges[i] is the match between keyframes_[i] and keyframes_[i + 1]
  std::vector<FeatureMatchEdge> match_edges_;

  std::vector<Landmark> landmarks_;

  // landmark_to_feature[i][j] is the no. of feature of ith landmark in
  // |landmarks_| in |keyframes_[j]|
  // The size should be [size of landmarks][size of keyframes]
  std::vector<std::unordered_map<int, int> > landmark_to_feature_;
  // Temporary landmarks that generated from feature matches
  std::vector<std::unordered_map<int, int> > uninited_landmark_to_feature_;
  // Last frame feature id to uninited landmarks
  std::unordered_map<int, int> last_frame_feature_to_uninited_landmark_;

  // feature_to_landmark[i][j] is the no. of landmark of ith feature in
  // keyframe[j]
  // The size should be [size of keyframe][number of features in keyframe i]
  std::vector<std::unordered_map<int, int> > feature_to_landmark_;

  int min_pnp_matches_;
};

}  // vio

#endif
