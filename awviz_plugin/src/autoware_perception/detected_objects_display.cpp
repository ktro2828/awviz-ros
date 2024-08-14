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

#include "awviz_plugin/autoware_perception/detected_objects_display.hpp"

#include <rerun.hpp>

#include <vector>

namespace awviz_plugin
{
DetectedObjectsDisplay::DetectedObjectsDisplay()
: awviz_common::RosTopicDisplay<autoware_perception_msgs::msg::DetectedObjects>()
{
}

void DetectedObjectsDisplay::logToStream(
  autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  std::vector<rerun::Position3D> centers;
  std::vector<rerun::HalfSize3D> sizes;
  std::vector<rerun::Rotation3D> rotations;
  std::vector<rerun::components::ClassId> class_ids;
  for (const auto & object : msg->objects) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const auto & dimensions = object.shape.dimensions;

    centers.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    sizes.emplace_back(dimensions.x, dimensions.y, dimensions.z);
    rotations.emplace_back(rerun::Quaternion::from_wxyz(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
    class_ids.emplace_back(static_cast<uint16_t>(object.classification.front().label));
  }

  stream_->log(
    property_.entity(), rerun::Boxes3D::from_centers_and_half_sizes(centers, sizes)
                          .with_rotations(rotations)
                          .with_class_ids(class_ids));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::DetectedObjectsDisplay, awviz_common::Display);