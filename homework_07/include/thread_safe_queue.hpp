#pragma once

#include <mutex>
#include <optional>
#include <queue>

template <typename T>
class ThreadSafeQueue {
public:
  void push(T value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(std::move(value));
  }

  bool tryPop(T &out)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return false;
    }
    out = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<T> empty;
    queue_.swap(empty);
  }

  std::optional<T> latestValue() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    return queue_.back();
  }

private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
};
