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

#include "awviz_plugin/autoware_perception/predicted_objects_display.hpp"

#include <rerun.hpp>
#include <rerun/archetypes/line_strips3d.hpp>

#include <vector>

namespace awviz_plugin
{
namespace
{
/**
 * @brief Convert predicted paths to the sequence of waypoints.
 *
 * @param path Sequence of predicted poses.
 * @return std::vector<rerun::Position3D>
 */
std::vector<rerun::Position3D> to_waypoints(
  const autoware_perception_msgs::msg::PredictedPath & path)
{
  std::vector<rerun::Position3D> waypoints;
  waypoints.reserve(path.path.size());
  for (const auto & pose : path.path) {
    waypoints.emplace_back(rerun::Position3D(pose.position.x, pose.position.y, pose.position.z));
  }
  return waypoints;
}

}  // namespace

PredictedObjectsDisplay::PredictedObjectsDisplay()
: awviz_common::RosTopicDisplay<autoware_perception_msgs::msg::PredictedObjects>()
{
}

void PredictedObjectsDisplay::log_message(
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  log_timestamp(rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec));

  const auto entity_path = resolve_entity_path(msg->header.frame_id);
  if (!entity_path) {
    warn_missing_entity(msg->header.frame_id);
    return;
  }

  std::vector<rerun::LineStrip3D> linestrips;
  std::vector<rerun::components::ClassId> class_ids;
  for (const auto & object : msg->objects) {
    const uint16_t class_id = object.classification.empty()
                                ? 0U
                                : static_cast<uint16_t>(object.classification.front().label);
    for (const auto & path : object.kinematics.predicted_paths) {
      if (path.path.empty()) {
        continue;
      }
      const auto waypoints = to_waypoints(path);

      linestrips.emplace_back(rerun::LineStrip3D(waypoints));
      class_ids.emplace_back(class_id);
    }
  }

  stream_->log(entity_path.value(), rerun::LineStrips3D(linestrips).with_class_ids(class_ids));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::PredictedObjectsDisplay, awviz_common::Display);
