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

#include "awviz_plugin/pointcloud/pointcloud2_display.hpp"

#include "awviz_plugin/color.hpp"

#include <rerun.hpp>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>

namespace awviz_plugin
{
PointCloud2Display::PointCloud2Display()
: awviz_common::RosTopicDisplay<sensor_msgs::msg::PointCloud2>()
{
}

void PointCloud2Display::log_message(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  auto isValidDataType = [](uint8_t datatype) {
    return datatype == sensor_msgs::msg::PointField::FLOAT32;
  };

  const auto entity_path = property_.entity(msg->header.frame_id);
  if (!entity_path) {
    stream_->log(property_.topic(), rerun::TextLog("There is no corresponding entity path"));
    return;
  }

  bool has_x{false}, has_y{false}, has_z{false};
  for (const auto & field : msg->fields) {
    if (field.name == "x") {
      if (!isValidDataType(field.datatype)) {
        stream_->log(entity_path.value(), rerun::TextLog("Only FLOAT32 x field supported"));
        return;
      }
      has_x = true;
    } else if (field.name == "y") {
      if (!isValidDataType(field.datatype)) {
        stream_->log(entity_path.value(), rerun::TextLog("Only FLOAT32 y field supported"));
        return;
      }
      has_y = true;
    } else if (field.name == "z") {
      if (!isValidDataType(field.datatype)) {
        stream_->log(entity_path.value(), rerun::TextLog("Only FLOAT32 z field supported"));
        return;
      }
      has_z = true;
    }
  }

  if (!has_x || !has_y || !has_z) {
    stream_->log(
      entity_path.value(), rerun::TextLog("Currently only PointCloud2 with x/y/z are supported"));
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> itr_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> itr_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> itr_z(*msg, "z");

  std::vector<rerun::Position3D> points(msg->width * msg->height);
  std::vector<float> distances(msg->width * msg->height);
  for (; itr_x != itr_x.end(); ++itr_x, ++itr_y, ++itr_z) {
    points.emplace_back(*itr_x, *itr_y, *itr_z);
    distances.emplace_back(std::hypot(*itr_x, *itr_y, *itr_z));
  }

  auto colors = colormap(distances);
  stream_->log(entity_path.value(), rerun::Points3D(points).with_colors(colors));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::PointCloud2Display, awviz_common::Display);
