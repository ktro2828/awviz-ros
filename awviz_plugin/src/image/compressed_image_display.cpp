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

#include "awviz_plugin/image/compressed_image_display.hpp"

#include "awviz_plugin/image/collection_adapter.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>

namespace awviz_plugin
{
CompressedImageDisplay::CompressedImageDisplay()
: awviz_common::RosTopicDisplay<sensor_msgs::msg::CompressedImage>()
{
}

void CompressedImageDisplay::logToStream(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  if (msg->format.find("jpeg")) {
    auto img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    stream_->log(property_.entity(), rerun::Image(tensorShape(img), rerun::TensorBuffer::u8(img)));
  } else {
    auto img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    cv::Mat depth;
    img.convertTo(depth, CV_32FC1);

    stream_->log(
      property_.entity(), rerun::DepthImage(
                            {static_cast<size_t>(depth.rows), static_cast<size_t>(depth.cols)},
                            rerun::TensorBuffer::f32(depth))
                            .with_meter(1.0));
  }
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::CompressedImageDisplay, awviz_common::Display);
