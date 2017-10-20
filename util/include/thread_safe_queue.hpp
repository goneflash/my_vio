#ifndef THREAD_SAFE_QUEUE_HPP_
#define THREAD_SAFE_QUEUE_HPP_

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include <iostream>

namespace vio {

template <typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() = default;
  ThreadSafeQueue(const ThreadSafeQueue &) = delete;

  // To make sure the size stays the same. Return the mutex and release when
  // done with the queue.
  // TODO: This is definitly not a smart way.
  std::unique_lock<std::mutex> size(size_t &size) {
    std::unique_lock<std::mutex> tmp_lock(mutex_);
    size = queue_.size();
    std::cout << "Now size: " << size << std::endl;
    return std::move(tmp_lock);
  }

  // TODO：What if unique_ptr object, rvalue stuff.
  void Push(const T &data) {
    std::unique_lock<std::mutex> tmp_lock(mutex_);
    queue_.push(data);

    tmp_lock.unlock();
    cond_.notify_one();
  }

  void Push(T &&data) {
    std::unique_lock<std::mutex> tmp_lock(mutex_);
    queue_.push(std::move(data));

    tmp_lock.unlock();
    cond_.notify_one();
  }

  /*
  T front() {
    std::unique_lock<std::mutex> tmp_lock(mutex_);
    while (queue_.empty()) {
      cond_.wait(tmp_lock);
    }
    auto data = queue_.front();
  }
  */

  /*
  void Pop(T &data) {
    std::unique_lock<std::mutex> tmp_lock(mutex_);
    while (queue_.empty()) {
      cond_.wait(tmp_lock);
    }
    data = queue_.front();
    queue_.pop();
  }
  */

  T Pop() {
    std::unique_lock<std::mutex> tmp_lock(mutex_);
    while (queue_.empty()) {
      cond_.wait(tmp_lock);
    }
    T data = std::move(queue_.front());
    queue_.pop();
    return std::move(data);
  }

  T Pop(std::unique_lock<std::mutex> tmp_lock) {
    while (queue_.empty()) {
      cond_.wait(tmp_lock);
    }
    T data = std::move(queue_.front());
    queue_.pop();
    return std::move(data);
  }

 private:
  std::queue<T> queue_;
  std::mutex mutex_;
  // Must work with unique_lock<mutex>
  std::condition_variable cond_;
};

}  // vio

#endif
