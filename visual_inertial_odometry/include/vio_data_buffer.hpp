#ifndef VIO_DATA_BUFFER_HPP_
#define VIO_DATA_BUFFER_HPP_

#include <iostream>

#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "thread_safe_queue.hpp"

namespace vio {

/*
 * Used to hold the data (image & imu data) for processing.
 * Will drop data if the processing is slow.
 */
class VIODataBuffer {
 public:
  // TODO: Move buffer size as parameters?
  VIODataBuffer()
      : image_buffer_size_(10),
        image_total_num_(0),
        image_dropped_num_(0),
        imu_buffer_size_(50),
        imu_total_num_(0),
        imu_dropped_num_(0) {}

  void AddImageData(cv::Mat &img) {
    image_total_num_++;
    while (true) {
      size_t size = 0;
      auto tmp_lock = image_buffer_.size(size);
      if (size >= image_buffer_size_) {
        // TODO: Drop smartly.
        image_buffer_.Pop(std::move(tmp_lock));
        std::cout << "Popped an image.\n";
        image_dropped_num_++;
      } else {
        tmp_lock.unlock();
        image_buffer_.Push(img);
        std::cout << "Pushed an image.\n";
        break;
      }
    }
  }

  void AddImuData() {}

  cv::Mat GetImageData() { return image_buffer_.Pop(); }

  // TODO: Interpolate imu data to get synchronous data.
  bool GetLatestDataComb() {}

 private:
  // Image data buffer.
  int image_buffer_size_;
  int image_total_num_;
  int image_dropped_num_;

  ThreadSafeQueue<cv::Mat> image_buffer_;

  // IMU data buffer.
  int imu_buffer_size_;
  int imu_total_num_;
  int imu_dropped_num_;
};
}

#endif  // VIO_DATA_BUFFER_HPP_
