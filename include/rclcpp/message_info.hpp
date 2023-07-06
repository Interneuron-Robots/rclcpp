/*
 * @Description: 
 * @Author: Sauron
 * @Date: 2023-05-16 17:07:07
 * @LastEditTime: 2023-07-05 16:09:44
 * @LastEditors: Sauron
 */
// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__MESSAGE_INFO_HPP_
#define RCLCPP__MESSAGE_INFO_HPP_

#include "rmw/types.h"

#include "rclcpp/visibility_control.hpp"

#ifdef INTERNEURON
#include "interneuron_lib/time_point_manager.hpp"
#include<map>
#include<string>
#include<iostream>
#endif
namespace rclcpp
{
struct TP_Info{
  uint64_t this_sample_time_;
  uint64_t last_sample_time_;
  uint64_t remain_time_;// with the reference time in timepoint and remain_time in message_info, you can know the deadline
  TP_Info(uint64_t this_sample_time, uint64_t last_sample_time, uint64_t remain_time):this_sample_time_(this_sample_time), last_sample_time_(last_sample_time), remain_time_(remain_time){}
  };
/// Additional meta data about messages taken from subscriptions.
class RCLCPP_PUBLIC MessageInfo
{
public:
  /// Default empty constructor.
  MessageInfo() = default;

  /// Conversion constructor, which is intentionally not marked as explicit.
  /**
   * \param[in] rmw_message_info message info to initialize the class
   */
  // cppcheck-suppress noExplicitConstructor
  MessageInfo(const rmw_message_info_t & rmw_message_info);  // NOLINT(runtime/explicit)

  virtual ~MessageInfo();

  /// Return the message info as the underlying rmw message info type.
  const rmw_message_info_t &
  get_rmw_message_info() const;

  /// Return the message info as the underlying rmw message info type.
  rmw_message_info_t &
  get_rmw_message_info();


  #ifdef INTERNEURON
  void set_tp_info(std::string sensor_name, uint64_t this_sample_time, uint64_t last_sample_time, uint64_t remain_time);
  uint64_t get_last_sample_time(std::string sensor_name) const;
  uint64_t get_remain_time(std::string sensor_name) const;
  
  std::map<std::string, TP_Info> tp_infos_;//key is the sensor name
   #endif

private:
  rmw_message_info_t rmw_message_info_;
};

}  // namespace rclcpp

#endif  // RCLCPP__MESSAGE_INFO_HPP_
