// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_
#define RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>

#include "rclcpp/executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/visibility_control.hpp"

#ifdef PICAS
#include <rclcpp/cb_sched.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/syscall.h>
#include <linux/sched.h>

// for sched_deadline
#include <pthread.h>
#define gettid() syscall(__NR_gettid)
struct sched_attr {
    int32_t size;

    int32_t sched_policy;
    int64_t sched_flags;

    /* SCHED_NORMAL, SCHED_BATCH */
    int32_t sched_nice;

    /* SCHED_FIFO, SCHED_RR */
    int32_t sched_priority;

    /* SCHED_DEADLINE (nsec) */
    int64_t sched_runtime;
    int64_t sched_deadline;
    int64_t sched_period;
};
#endif


namespace rclcpp
{
namespace executors
{

class MultiThreadedExecutor : public rclcpp::Executor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MultiThreadedExecutor)

  /// Constructor for MultiThreadedExecutor.
  /**
   * For the yield_before_execute option, when true std::this_thread::yield()
   * will be called after acquiring work (as an AnyExecutable) and
   * releasing the spinning lock, but before executing the work.
   * This is useful for reproducing some bugs related to taking work more than
   * once.
   *
   * \param options common options for all executors
   * \param number_of_threads number of threads to have in the thread pool,
   *   the default 0 will use the number of cpu cores found instead
   * \param yield_before_execute if true std::this_thread::yield() is called
   * \param timeout maximum time to wait
   */
  RCLCPP_PUBLIC
  explicit MultiThreadedExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  virtual ~MultiThreadedExecutor();

  /**
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void
  spin() override;

  RCLCPP_PUBLIC
  size_t
  get_number_of_threads();

#ifdef PICAS
  std::vector<int> cpus;
  struct sched_attr rt_attr;
#endif


protected:
  RCLCPP_PUBLIC
  void
  run(size_t this_thread_number);

private:
  RCLCPP_DISABLE_COPY(MultiThreadedExecutor)

  std::mutex wait_mutex_;
  size_t number_of_threads_;
  bool yield_before_execute_;
  std::chrono::nanoseconds next_exec_timeout_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__MULTI_THREADED_EXECUTOR_HPP_
