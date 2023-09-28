// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
#define RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_

#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp/experimental/buffers/buffer_implementation_base.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

#ifdef INTERNEURON
#include "rclcpp/message_info.hpp"
#endif

namespace rclcpp
{
namespace experimental
{
namespace buffers
{

/// Store elements in a fixed-size, FIFO buffer
/**
 * All public member functions are thread-safe.
 */
template<typename BufferT>
class RingBufferImplementation : public BufferImplementationBase<BufferT>
{
public:
#ifdef INTERNEURON
  explicit RingBufferImplementation(size_t capacity)
  : capacity_(capacity),
    ring_buffer_(capacity),
    message_info_buffer_(capacity),
    write_index_(capacity_ - 1),
    read_index_(0),
    size_(0)
  {
    if (capacity == 0) {
      throw std::invalid_argument("capacity must be a positive, non-zero value");
    }
  }
  #else
explicit RingBufferImplementation(size_t capacity)
  : capacity_(capacity),
    ring_buffer_(capacity),
    write_index_(capacity_ - 1),
    read_index_(0),
    size_(0)
  {
    if (capacity == 0) {
      throw std::invalid_argument("capacity must be a positive, non-zero value");
    }
  }
  #endif

  virtual ~RingBufferImplementation() {}

  /// Add a new element to store in the ring buffer
  /**
   * This member function is thread-safe.
   *
   * \param request the element to be stored in the ring buffer
   */
  void enqueue(BufferT request)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    write_index_ = next_(write_index_);
    ring_buffer_[write_index_] = std::move(request);

    if (is_full_()) {
      read_index_ = next_(read_index_);
    } else {
      size_++;
    }
  }

  /// Remove the oldest element from ring buffer
  /**
   * This member function is thread-safe.
   *
   * \return the element that is being removed from the ring buffer
   */
  BufferT dequeue()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_data_()) {
      return BufferT();
    }

    auto request = std::move(ring_buffer_[read_index_]);
    read_index_ = next_(read_index_);

    size_--;

    return request;
  }
#ifdef INTERNEURON
/// Add a new element to store in the ring buffer
  /**
   * This member function is thread-safe.
   *
   * \param request the element to be stored in the ring buffer
   */
  bool enqueue(BufferT request, std::unique_ptr<rclcpp::MessageInfo> message_info)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (if_full_())
    {
      if(this->reliable_)return false;
      auto dump_info = std::move(message_info_abuffer_[read_index_]);
      read_index_ = next_(read_index_);

      write_index_ = next_(write_index_);
      ring_buffer_[write_index_] = std::move(request);
      message_info_buffer_[write_index_] = std::move(message_info);
      message_info_buffer_[read_index_]->merge_another_message_info(*dump_info);
    }else{
      write_index_ = next_(write_index_);
      ring_buffer_[write_index_] = std::move(request);
      message_info_buffer_[write_index_] = std::move(message_info);
      size_++;
    }
    
    return true;
  }

  /// Remove the oldest element from ring buffer
  /**
   * This member function is thread-safe.
   *
   * \return the element that is being removed from the ring buffer
   */
  std::pair<BufferT, std::unique_ptr<rclcpp::MessageInfo>> dequeue_with_message_info()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_data_()) {
      //return std::make_pair(BufferT(), std::make_unique<rclcpp::MessageInfo>());
      return std::make_pair(BufferT(), nullptr);
    }
    auto old_index = read_index_;
    //auto request = std::move(ring_buffer_[read_index_]);
    //auto message_info = std::move(message_info_buffer_[read_index_]);
    read_index_ = next_(read_index_);

    size_--;

    return std::make_pair(std::move(ring_buffer_[old_index]), std::move(message_info_buffer_[old_index]));
  }

  // following funcs are not thread-safe, you should use lock() to protect them
  // these funcs are used to return the optimal msg to make the fused msg


  // find_message is usually followed by dequeue_with_message_info
  size_t find_message(uint64_t& pivot_earliest_time, uint64_y& pivot_latest_time, const uint64_t interval_bound, bool disparity_optimal){
    if(!has_data_())return -1;
    if(interval_bound == NO_INTERVAL_LIMIT && disparity_optimal == false)return read_index_;// return the front
    auto pivot_interval = pivot_latest_time - pivot_earliest_time;
    if(pivot_interval > interval_bound)return -2;//should not happen, a big mistake

    size_t index = read_index_;
    size_t offset = 0;
    size_t min_index = -1;
    uint64_t min_interval = NO_INTERVAL_LIMIT;
    while(offset < size_){
      auto tmp_earliest_time = message_info_buffer_[index]->earliest_this_sample_time();
      auto tmp_latest_time = message_info_buffer_[index]->latest_this_sample_time();
      auto tmp_interval = tmp_latest_time - tmp_earliest_time;
      if(tmp_interval > interval_bound)return -2;// wont continue since it is a big mistake
      auto latest = std::max(pivot_latest_time, tmp_latest_time);
      auto earliest = std::min(pivot_earliest_time, tmp_earliest_time);
      auto interval = latest - earliest;
      if(min_interval>interval && interval <= interval_bound){
        min_interval = interval;
        min_index = index;
        if(!disparity_optimal)return min_index;//find one, just return this earliest msg
      }//if min_interval == interval, we will still use the earlier one, so no need to update min_index
      
      offset++;
      index = next_(index);
    }
    return min_index;// if no msg is found, will still return -1
  }

  // this function will return the msg in the index position and dump earlier msgs, the returned msg's message_info will be updated
  std::pair<BufferT, std::unique_ptr<rclcpp::MessageInfo>> dequeue_with_message_info(size_t index)
  {
    index = index % capacity_;
    if ((!has_data_())||(write_index_ >= read_index_ && (index > write_index_||index<read_index_))||(write_index_ < read_index_ && index > write_index_ && index < read_index_)) {
      //return std::make_pair(BufferT(), std::make_unique<rclcpp::MessageInfo>());
      return std::make_pair(BufferT(), nullptr);
    }
    message_info_buffer_[index]->merge_another_message_info(*message_info_buffer_[read_index_]);
    size_--;
    if(index>=read_index_){
      size_ = size_ - (index - read_index_);
    }else{
      size_ = size_ - (capacity_ - read_index_ + index);
    }

    read_index_ = next_(index);
    return std::make_pair(std::move(ring_buffer_[index]), std::move(message_info_buffer_[index]));
  }

  void lock()
  {
    mutex_.lock();
  }

  void unlock()
  {
    mutex_.unlock();
  }
#endif
  /// Get the next index value for the ring buffer
  /**
   * This member function is thread-safe.
   *
   * \param val the current index value
   * \return the next index value
   */
  inline size_t next(size_t val)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return next_(val);
  }

  /// Get if the ring buffer has at least one element stored
  /**
   * This member function is thread-safe.
   *
   * \return `true` if there is data and `false` otherwise
   */
  inline bool has_data() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_data_();
  }

  /// Get if the size of the buffer is equal to its capacity
  /**
   * This member function is thread-safe.
   *
   * \return `true` if the size of the buffer is equal is capacity
   * and `false` otherwise
   */
  inline bool is_full() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return is_full_();
  }

  void clear() {}

private:
  /// Get the next index value for the ring buffer
  /**
   * This member function is not thread-safe.
   *
   * \param val the current index value
   * \return the next index value
   */
  inline size_t next_(size_t val)
  {
    return (val + 1) % capacity_;
  }

  /// Get if the ring buffer has at least one element stored
  /**
   * This member function is not thread-safe.
   *
   * \return `true` if there is data and `false` otherwise
   */
  inline bool has_data_() const
  {
    return size_ != 0;
  }

  /// Get if the size of the buffer is equal to its capacity
  /**
   * This member function is not thread-safe.
   *
   * \return `true` if the size of the buffer is equal is capacity
   * and `false` otherwise
   */
  inline bool is_full_() const
  {
    return size_ == capacity_;
  }

  size_t capacity_;

  std::vector<BufferT> ring_buffer_;
  #ifdef INTERNEURON
  std::vector<std::unique_ptr<rclcpp::MessageInfo>> message_info_buffer_;
  #endif

  size_t write_index_;
  size_t read_index_;
  size_t size_;

  mutable std::mutex mutex_;
};

}  // namespace buffers
}  // namespace experimental
}  // namespace rclcpp

#endif  // RCLCPP__EXPERIMENTAL__BUFFERS__RING_BUFFER_IMPLEMENTATION_HPP_
