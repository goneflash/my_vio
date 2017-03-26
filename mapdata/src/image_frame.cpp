#include "image_frame.hpp"

#include <math.h>

namespace vio {

ImageFrame::ImageFrame(const cv::Mat &image)
    : has_grid_keypoints_(false), grid_width_size_(10), grid_height_size_(10) {
  image.copyTo(image_);
  grid_width_index_range_ = image_.size().width / grid_width_size_ - 1;
  grid_height_index_range_ = image_.size().height / grid_height_size_ - 1;

  grid_keypoints_index_.resize(grid_width_index_range_ + 1);
  for (int i = 0; i < grid_width_index_range_ + 1; ++i)
    grid_keypoints_index_[i].resize(grid_height_index_range_ + 1);
}

bool ImageFrame::CreateGridKeypointIndex() {
  if (!keypoints_.size()) return false;

  keypoint_bin_coords_.resize(keypoints_.size());
  for (int kp_id = 0; kp_id < keypoints_.size(); ++kp_id) {
    int x_grid_index = ((int)keypoints_[kp_id].pt.x - 1) / grid_width_size_;
    int y_grid_index = ((int)keypoints_[kp_id].pt.y - 1) / grid_height_size_;
    grid_keypoints_index_[x_grid_index][y_grid_index].push_back(kp_id);
    keypoint_bin_coords_[kp_id].x = x_grid_index;
    keypoint_bin_coords_[kp_id].y = y_grid_index;
  }
  has_grid_keypoints_ = true;
}

bool ImageFrame::GetNeighborKeypointsInRadius(
    const cv::KeyPoint &query, double dist_thresh,
    std::vector<int> &candidates_id) const {
  if (!has_grid_keypoints_) return false;

  int x_grid_index = ((int)query.pt.x - 1) / grid_width_size_;
  int y_grid_index = ((int)query.pt.y - 1) / grid_height_size_;

  int width_search_index_range = (int)ceil(dist_thresh / grid_width_size_);
  int height_search_index_range = (int)ceil(dist_thresh / grid_height_size_);

  int min_width_index = std::max(x_grid_index - width_search_index_range, 0);
  int min_height_index = std::max(y_grid_index - height_search_index_range, 0);
  int max_width_index = std::min(x_grid_index + width_search_index_range,
                                 grid_width_index_range_);
  int max_height_index = std::min(y_grid_index + height_search_index_range,
                                  grid_height_index_range_);

  candidates_id.clear();
  for (int w = min_width_index; w <= max_width_index; ++w) {
    for (int h = min_height_index; h <= max_height_index; ++h) {
      // TODO: Added dist threshold
      // TODO: Add unit test
      for (int i = 0; i < grid_keypoints_index_[w][h].size(); ++i) {
        const int kp_id = grid_keypoints_index_[w][h][i];
        candidates_id.push_back(kp_id);
      }
    }
  }

  return true;
}

void ImageFrame::SetGridSize(int width, int height) {
  grid_width_size_ = width;
  grid_height_size_ = height;
}

}  // vio
