/*
 * @Description: 
 * @Author: Sauron
 * @Date: 2023-05-16 17:07:07
 * @LastEditTime: 2023-06-02 15:11:41
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

#include "rclcpp/message_info.hpp"

namespace rclcpp
{

  MessageInfo::MessageInfo(const rmw_message_info_t &rmw_message_info)
      : rmw_message_info_(rmw_message_info)
  {
  }

  MessageInfo::~MessageInfo()
  {
  }

  const rmw_message_info_t &
  MessageInfo::get_rmw_message_info() const
  {
    return rmw_message_info_;
  }

  rmw_message_info_t &
  MessageInfo::get_rmw_message_info()
  {
    return rmw_message_info_;
  }

#ifdef INTERNEURON
  uint64_t
  MessageInfo::get_start_time()
  {
    return start_time_;
  }

  uint64_t
  MessageInfo::get_remain_time()
  {
    return remain_time_;
  }

  void
  MessageInfo::set_start_time(uint64_t start_time)
  {
    start_time_ = start_time;
  }

  void
  MessageInfo::set_remain_time(uint64_t remain_time)
  {
    remain_time_ = remain_time;
  }

#endif

} // namespace rclcpp
