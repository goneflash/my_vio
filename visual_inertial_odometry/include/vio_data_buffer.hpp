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
struct VIODataBufferStats {
 public:
  VIODataBufferStats() : received_count(0), dropped_count(0) {}
  int received_count;
  int dropped_count;

  void Print() const {
    std::cout << "Received " << received_count << ", dropped " << dropped_count
              << std::endl;
  }
};

class VIODataBuffer {
 public:
  // TODO: Move buffer size as parameters?
  VIODataBuffer() : image_buffer_size_(10), imu_buffer_size_(50) {}

  void AddImageData(cv::Mat &img) {
    image_buffer_stats_.received_count++;
    while (true) {
      size_t size = 0;
      auto tmp_lock = image_buffer_.size(size);
      if (size >= image_buffer_size_) {
        // TODO: Drop smartly.
        image_buffer_.Pop(std::move(tmp_lock));
        image_buffer_stats_.dropped_count++;
      } else {
        tmp_lock.unlock();
        image_buffer_.Push(img);
        break;
      }
    }
  }

  void AddImuData() {}

  cv::Mat GetImageData() { return image_buffer_.Pop(); }

  // TODO: Interpolate imu data to get synchronous data.
  bool GetLatestDataComb() {}

  const VIODataBufferStats &image_buffer_stats() const {
    return image_buffer_stats_;
  }

 private:
  // Image data buffer.
  int image_buffer_size_;
  ThreadSafeQueue<cv::Mat> image_buffer_;
  // TODO: Do not need mutex until it is used in multithread.
  VIODataBufferStats image_buffer_stats_;

  // IMU data buffer.
  int imu_buffer_size_;
  // ThreadSafeQueue<cv::Mat> imu_buffer_;
  VIODataBufferStats imu_buffer_stats_;
};
}

#endif  // VIO_DATA_BUFFER_HPP_
