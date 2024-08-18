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

#include "awviz_plugin/image/collection_adapter.hpp"

#include <rerun.hpp>

#include <cv_bridge/cv_bridge.h>

namespace awviz_plugin
{
ImageDisplay::ImageDisplay() : awviz_common::RosTopicDisplay<sensor_msgs::msg::Image>()
{
}

void ImageDisplay::log_message(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  const auto entity_path = property_.entity_without_topic(msg->header.frame_id);
  if (!entity_path) {
    stream_->log(property_.topic(), rerun::TextLog("There is no corresponding entity path"));
    return;
  }

  if (msg->encoding == "16UC1") {
    auto img = cv_bridge::toCvCopy(msg)->image;
    stream_->log(
      entity_path.value(), rerun::DepthImage(
                             {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                             rerun::TensorBuffer::u16(img))
                             .with_meter(1000));
  } else if (msg->encoding == "32FC1") {
    auto img = cv_bridge::toCvCopy(msg)->image;

    stream_->log(
      entity_path.value(), rerun::DepthImage(
                             {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                             rerun::TensorBuffer::f32(img))
                             .with_meter(1.0));
  } else {
    auto img = cv_bridge::toCvCopy(msg, "rgb8")->image;

    stream_->log(
      entity_path.value(), rerun::Image(tensor_shape(img), rerun::TensorBuffer::u8(img)));
  }
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::ImageDisplay, awviz_common::Display);
