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

#include <vector>

namespace awviz_plugin
{
PredictedObjectsDisplay::PredictedObjectsDisplay()
: awviz_common::RosTopicDisplay<autoware_perception_msgs::msg::PredictedObjects>()
{
}

void PredictedObjectsDisplay::log_message(
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  const auto entity_path = property_.entity(msg->header.frame_id);
  if (!entity_path) {
    stream_->log(property_.topic(), rerun::TextLog("There is no corresponding entity path"));
    return;
  }

  std::vector<rerun::Position3D> centers;
  std::vector<rerun::HalfSize3D> sizes;
  std::vector<rerun::Rotation3D> rotations;
  std::vector<rerun::components::ClassId> class_ids;
  std::vector<rerun::LineStrip3D> paths;
  for (const auto & object : msg->objects) {
    const auto & init_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & dimensions = object.shape.dimensions;
    centers.emplace_back(init_pose.position.x, init_pose.position.y, init_pose.position.z);
    sizes.emplace_back(dimensions.x, dimensions.y, dimensions.z);
    rotations.emplace_back(rerun::Quaternion::from_wxyz(
      init_pose.orientation.w, init_pose.orientation.x, init_pose.orientation.y,
      init_pose.orientation.z));
    class_ids.emplace_back(static_cast<uint16_t>(object.classification.front().label));

    for (const auto & path : object.kinematics.predicted_paths) {
      std::vector<rerun::Vec3D> waypoints;
      for (const auto & point : path.path) {
        waypoints.emplace_back(point.position.x, point.position.y, point.position.z);
      }
      paths.emplace_back(rerun::LineStrip3D(waypoints));
    }
  }

  stream_->log(
    entity_path.value(),
    rerun::Boxes3D::from_centers_and_half_sizes(centers, sizes)
      .with_rotations(rotations)
      .with_class_ids(class_ids),
    rerun::LineStrips3D(paths));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::PredictedObjectsDisplay, awviz_common::Display);
