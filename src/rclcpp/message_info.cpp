/*
 * @Description: 
 * @Author: Sauron
 * @Date: 2023-05-16 17:07:07
 * @LastEditTime: 2023-09-26 11:55:16
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
MessageInfo::MessageInfo(const std::vector<std::string>&sensor_names){
  for(auto&sensor_name:sensor_names){
    this->tp_infos_.insert(std::make_pair(sensor_name, interneuron::TP_Info()));
  }
}

  void MessageInfo::update_TP_Info(std::string sensor_name, uint64_t this_sample_time, uint64_t last_sample_time, uint64_t remain_time){
    auto tp_info = this->tp_infos_.find(sensor_name);
    if (tp_info == this->tp_infos_.end()){
      this->tp_infos_.insert(std::make_pair(sensor_name, interneuron::TP_Info(this_sample_time, last_sample_time, remain_time)));
    }
    else{
      tp_info->second.this_sample_time_ = this_sample_time;
      tp_info->second.last_sample_time_ = last_sample_time;
      tp_info->second.remain_time_ = remain_time;
    }
  }

  void MessageInfo::update_TP_Info(std::string sensor_name, interneuron::TP_Info tp_info){
    auto tp_info_ = this->tp_infos_.find(sensor_name);
    if (tp_info_ == this->tp_infos_.end()){
      this->tp_infos_.insert(std::make_pair(sensor_name, tp_info));
    }
    else{
      tp_info_->second = tp_info;
    }
  }

  interneuron::TP_Info& MessageInfo::get_TP_Info(std::string sensor_name){
    auto tp_info = this->tp_infos_.find(sensor_name);
    if (tp_info == this->tp_infos_.end()){
      #ifdef PRINT_DEBUG
      std::cout<<"[ERROR][MessageInfo::get_TP_Info] cannot find: "<<sensor_name<<"in the message_info"<<std::endl;
      #endif
      assert(false);
    }
    return tp_info->second;
  }

  void MessageInfo::merge_another_message_info(MessageInfo& another_message_info){
    for(auto& tp_info:another_message_info.tp_infos_){
      auto tp_info_ = this->tp_infos_.find(tp_info.first);
      if (tp_info_ == this->tp_infos_.end()){
        // add tp_infos from other sensors
        this->tp_infos_.insert(std::make_pair(tp_info.first, tp_info.second));
      }
      else{
        tp_info_->second.last_sample_time = tp_info.second.last_sample_time;
        tp_info_->second.remain_time = tp_info.second.remain_time;
      }
    }
  }

uint64_t MessageInfo::earliest_this_sample_time(){
  uint64_t earliest_this_sample_time = 0;
  for(auto& tp_info:this->tp_infos_){
    if(tp_info.second.this_sample_time_ < earliest_this_sample_time){
      earliest_this_sample_time = tp_info.second.this_sample_time_;
    }
  }
  return earliest_this_sample_time;
}

uint64_t MessageInfo::latest_this_sample_time(){
  uint64_t latest_this_sample_time = 0;
  for(auto& tp_info:this->tp_infos_){
    if(tp_info.second.this_sample_time_ > latest_this_sample_time){
      latest_this_sample_time = tp_info.second.this_sample_time_;
    }
  }
  return latest_this_sample_time;
}

#endif

} // namespace rclcpp
