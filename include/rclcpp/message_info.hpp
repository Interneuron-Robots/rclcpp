/*
 * @Description: 
 * @Author: Sauron
 * @Date: 2023-05-16 17:07:07
 * @LastEditTime: 2023-09-27 09:46:46
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
#include "interneuron_lib/time_point.hpp"
#include<vector>
#endif
namespace rclcpp
{
#ifdef INTERNEURON
#include<limits>
const uint64_t NO_INTERVAL_LIMIT = std::numeric_limits<uint64_t>::max();
#endif
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
MessageInfo(const std::vector<std::string>&sensor_names);

  void update_TP_Info(std::string sensor_name, uint64_t this_sample_time, uint64_t last_sample_time, uint64_t remain_time);
  void update_TP_Info(std::string sensor_name, interneuron::TP_Info tp_info);
  interneuron::TP_Info& get_TP_Info(std::string sensor_name);

  // merge function wont check whether these two message_infos are valid to be merged, it only do the work
  // for the same sensor, it will use the new msg's this_sample_time and the old msg's last_sample_time and remain_time
  // make sure that this msg is the new msg, and the msg to be merged is the old msg
  void merge_another_message_info(MessageInfo& another_message_info);
  
  //although we could carry these info, we get them during the execution
  uint64_t earliest_this_sample_time();
  uint64_t latest_this_sample_time();

  std::map<std::string, interneuron::TP_Info> tp_infos_;//key is the sensor name

  #endif

private:
  rmw_message_info_t rmw_message_info_;
};

}  // namespace rclcpp

#endif  // RCLCPP__MESSAGE_INFO_HPP_
