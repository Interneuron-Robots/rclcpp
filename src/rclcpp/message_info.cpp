/*
 * @Description: 
 * @Author: Sauron
 * @Date: 2023-05-16 17:07:07
 * @LastEditTime: 2023-07-11 17:26:30
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
#include <cassert>

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
  void MessageInfo::update_TP_Info(std::string sensor_name, uint64_t this_sample_time, uint64_t last_sample_time, uint64_t remain_time){
    auto tp_info = this->tp_infos_.find(sensor_name);
    if (tp_info == this->tp_infos_.end()){
      this->tp_infos_.insert(std::make_pair(sensor_name, TP_Info(this_sample_time, last_sample_time, remain_time)));
    }
    else{
      tp_info->second.this_sample_time_ = this_sample_time;
      tp_info->second.last_sample_time_ = last_sample_time;
      tp_info->second.remain_time_ = remain_time;
    }
  }

  void MessageInfo::updateTP_Info(std::string sensor_name, interneuron::TP_Info tp_info){
    auto tp_info_ = this->tp_infos_.find(sensor_name);
    if (tp_info_ == this->tp_infos_.end()){
      this->tp_infos_.insert(std::make_pair(sensor_name, tp_info));
    }
    else{
      tp_info_->second = tp_info;
    }
  }

  interneuron::TP_Info& get_TP_Info(std::string sensor_name){
    auto tp_info = this->tp_infos_.find(sensor_name);
    if (tp_info == this->tp_infos_.end()){
      #ifdef PRINT_DEBUG
      std::cout<<"[ERROR][MessageInfo::get_TP_Info] cannot find: "<<sensor_name<<"in the message_info"<<std::endl;
      #endif
      assert(false);
      return nullptr;
    }
    return tp_info->second;
  }

#endif

} // namespace rclcpp
