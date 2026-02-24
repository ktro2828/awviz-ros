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

#include "awviz_plugin/image/image_display.hpp"

#include "awviz_plugin/collection_adapter.hpp"

#include <rerun.hpp>

#include <cv_bridge/cv_bridge.h>

namespace awviz_plugin
{
ImageDisplay::ImageDisplay() : awviz_common::RosTopicDisplay<sensor_msgs::msg::Image>()
{
}

void ImageDisplay::log_message(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  log_timestamp(rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec));

  const auto entity_path = resolve_entity_path(msg->header.frame_id, false);
  if (!entity_path) {
    warn_missing_entity(msg->header.frame_id);
    return;
  }

  // TODO(ktro2828): Improve error handling by logging cv_bridge::Exception instead of silently
  // ignoring it.
  try {
    if (msg->encoding == "16UC1") {
      auto img = cv_bridge::toCvCopy(msg)->image;
      stream_->log(
        entity_path.value(),
        rerun::DepthImage(img.data, rerun::WidthHeight(img.cols, img.rows)).with_meter(1000));
    } else if (msg->encoding == "32FC1") {
      auto img = cv_bridge::toCvCopy(msg)->image;

      stream_->log(
        entity_path.value(),
        rerun::DepthImage(img.data, rerun::WidthHeight(img.cols, img.rows)).with_meter(1.0));
    } else {
      auto img = cv_bridge::toCvCopy(msg, "rgb8")->image;

      stream_->log(
        entity_path.value(), rerun::Image::from_rgb24(img, rerun::WidthHeight(img.cols, img.rows)));
    }
  } catch (const cv_bridge::Exception & e) {
  }
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::ImageDisplay, awviz_common::Display);
