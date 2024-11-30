// Copyright 2024 Kotaro Uetake.
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

#include "awviz_plugin/navsatfix/navsatfix_display.hpp"

#include <rerun.hpp>

namespace awviz_plugin
{
NavSatFixDisplay::NavSatFixDisplay() : awviz_common::RosTopicDisplay<sensor_msgs::msg::NavSatFix>()
{
}

void NavSatFixDisplay::log_message(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  const auto entity_path = property_.entity();
  stream_->log(entity_path, rerun::GeoPoints(rerun::LatLon(msg->latitude, msg->longitude)));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::NavSatFixDisplay, awviz_common::Display);
