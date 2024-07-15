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

#include "awviz/rerun_ros_interface.hpp"

#include "collection_adapters.hpp"
#include "color.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <cstddef>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

namespace awviz
{
void logPointCloud(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  size_t x_offset, y_offset, z_offset;
  bool has_x{false}, has_y{false}, has_z{false};
  for (const auto & field : msg->fields) {
    if (field.name == "x") {
      x_offset = field.offset;
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        stream.log(entity, rerun::TextLog("Only FLOAT32 x field supported"));
        return;
      }
      has_x = true;
    } else if (field.name == "y") {
      y_offset = field.offset;
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        stream.log(entity, rerun::TextLog("Only FLOAT32 y field supported"));
        return;
      }
      has_y = true;
    } else if (field.name == "z") {
      z_offset = field.offset;
      if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        stream.log(entity, rerun::TextLog("Only FLOAT32 z field supported"));
        return;
      }
      has_z = true;
    }
  }

  if (!has_x || !has_y || !has_z) {
    stream.log(entity, rerun::TextLog("Currently only PointCloud2 with x/y/z are supported"));
    return;
  }

  std::vector<rerun::Position3D> points(msg->width * msg->height);

  for (size_t i = 0; i < msg->height; ++i) {
    for (size_t j = 0; j < msg->width; ++j) {
      auto offset = i * msg->row_step + j * msg->point_step;
      rerun::Position3D position;
      std::memcpy(&position.xyz.xyz[0], &msg->data[offset + x_offset], sizeof(float));
      std::memcpy(&position.xyz.xyz[1], &msg->data[offset + y_offset], sizeof(float));
      std::memcpy(&position.xyz.xyz[2], &msg->data[offset + z_offset], sizeof(float));
      points.emplace_back(std::move(position));
    }
  }

  std::vector<float> values(msg->width * msg->height);
  auto colors = colormap(values);
  stream.log(entity, rerun::Points3D(points).with_colors(colors));
}

void logImage(
  const rerun::RecordingStream & stream, const std::string & entity,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  stream.set_time_seconds(
    "timestamp", rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  if (msg->encoding == "16UC1") {
    auto img = cv_bridge::toCvCopy(msg)->image;
    stream.log(
      entity, rerun::DepthImage(
                {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                rerun::TensorBuffer::u16(img))
                .with_meter(1000));
  } else if (msg->encoding == "32FC1") {
    auto img = cv_bridge::toCvCopy(msg)->image;

    stream.log(
      entity, rerun::DepthImage(
                {static_cast<size_t>(img.rows), static_cast<size_t>(img.cols)},
                rerun::TensorBuffer::f32(img))
                .with_meter(1.0));
  } else {
    auto img = cv_bridge::toCvCopy(msg, "rgb8")->image;
    stream.log(entity, rerun::Image(tensorShape(img), rerun::TensorBuffer::u8(img)));
  }
}

}  // namespace awviz
