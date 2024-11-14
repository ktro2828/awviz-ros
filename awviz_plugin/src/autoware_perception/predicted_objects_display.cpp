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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/convert.h>

#include <vector>

namespace awviz_plugin
{
namespace
{
rerun::Vec3D do_transform(
  const geometry_msgs::msg::Pose & init_pose, const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::TransformStamped transform;

  transform.transform.translation.x = init_pose.position.x;
  transform.transform.translation.y = init_pose.position.y;
  transform.transform.translation.z = init_pose.position.z;
  transform.transform.rotation.x = init_pose.orientation.x;
  transform.transform.rotation.y = init_pose.orientation.y;
  transform.transform.rotation.z = init_pose.orientation.z;
  transform.transform.rotation.w = init_pose.orientation.w;

  geometry_msgs::msg::Pose pose_out;
  tf2::doTransform(pose, pose_out, transform);
  return rerun::Vec3D(pose_out.position.x, pose_out.position.y, pose_out.position.z);
}

}  // namespace

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

  std::vector<rerun::LineStrip3D> paths;
  std::vector<rerun::components::ClassId> class_ids;
  for (const auto & object : msg->objects) {
    const auto & init_pose = object.kinematics.initial_pose_with_covariance.pose;

    const auto class_id = static_cast<uint16_t>(object.classification.front().label);

    for (const auto & path : object.kinematics.predicted_paths) {
      std::vector<rerun::Vec3D> waypoints;
      for (const auto & pose : path.path) {
        const auto & point = do_transform(init_pose, pose);
        waypoints.emplace_back(point);
      }
      paths.emplace_back(rerun::LineStrip3D(waypoints));
      class_ids.emplace_back(class_id);
    }
  }

  stream_->log(entity_path.value(), rerun::LineStrips3D(paths));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::PredictedObjectsDisplay, awviz_common::Display);
