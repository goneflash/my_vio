#ifndef VIO_KEYFRAME_
#define VIO_KEYFRAME_

#include <memory>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "cv2eigen_helper.hpp"
#include "image_frame.hpp"
#include "mapdata_types.hpp"

namespace vio {

struct CameraPose {
 public:
  CameraPose() : timestamp(0) {
    position << 0, 0, 0;
    orientation << 0, 0, 0, 1;
  }
  double timestamp;
  // In world frame.
  Eigen::Vector3d position;
  // In world frame.
  Eigen::Vector4d orientation;

  // TODO: Remove this. Only use eigen.
  cv::Mat R;
  cv::Vec3d t;
};

typedef int FeatureId;

struct Feature {
  Feature() : landmark_id(-1), measurement(-1, -1) {}
  LandmarkId landmark_id;
  FeatureMeasurement measurement;
  // TODO: Add descriptor;
};

class Keyframe {
 public:
  Keyframe(std::unique_ptr<ImageFrame> frame)
      : frame_id(CreateNewId<KeyframeId>()), pre_frame_id(-1) {
    // TODO: Not transfer ImageFrame, just copy the keypoints and descriptor.
    image_frame = std::move(frame);

    // Copy features.
    const auto &kps = image_frame->keypoints();
    for (int i = 0; i < kps.size(); ++i) {
      Feature new_feature;
      new_feature.measurement.x = kps[i].pt.x;
      new_feature.measurement.y = kps[i].pt.y;
      // TODO: Add descriptor;
      features[i] = new_feature;
    }
  }
  Keyframe() = delete;

  // TODO: inline
  void SetPose(const cv::Mat &R, const cv::Mat &t) {}

  KeyframeId frame_id;
  KeyframeId pre_frame_id;
  // FeatureId in current frame --> FeatureId in previous frame.
  std::unordered_map<int, int> match_to_pre_frame;

  std::unique_ptr<ImageFrame> image_frame;

  std::unordered_map<FeatureId, Feature> features;

  CameraPose pose;
};

typedef std::unordered_map<KeyframeId, std::unique_ptr<Keyframe>> Keyframes;
}  // namespace vio

#endif
