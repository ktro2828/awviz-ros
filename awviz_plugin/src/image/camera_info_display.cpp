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

#include "awviz_plugin/image/camera_info_display.hpp"

#include <awviz_common/display.hpp>

#include <array>

namespace awviz_plugin
{
CameraInfoDisplay::CameraInfoDisplay()
: awviz_common::RosTopicDisplay<sensor_msgs::msg::CameraInfo>()
{
}

void CameraInfoDisplay::log_message(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  const auto entity_path = property_.entity_without_topic(msg->header.frame_id);
  if (!entity_path) {
    stream_->log(property_.topic(), rerun::TextLog("There is no corresponding entity path"));
    return;
  }

  const std::array<float, 9> image_from_camera = {
    static_cast<float>(msg->k[0]), static_cast<float>(msg->k[3]), static_cast<float>(msg->k[6]),
    static_cast<float>(msg->k[1]), static_cast<float>(msg->k[4]), static_cast<float>(msg->k[7]),
    static_cast<float>(msg->k[2]), static_cast<float>(msg->k[5]), static_cast<float>(msg->k[8]),
  };

  stream_->log(
    entity_path.value(),
    rerun::Pinhole(image_from_camera)
      .with_resolution(static_cast<int>(msg->width), static_cast<int>(msg->height)));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::CameraInfoDisplay, awviz_common::Display);
