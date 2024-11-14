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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>

#include <vector>

namespace awviz_plugin
{
namespace
{
/**
 * @brief Transform pose from `init_pose` origin coordinate to world coordinate.
 *
 * @param origin Pose origin.
 * @param pose Pose which is with respect to `origin` origin coordinate.
 * @return rerun::Vec3D
 */
rerun::Vec3D do_transform(
  const geometry_msgs::msg::Pose & origin, const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion oq;
  oq.setValue(
    origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);

  tf2::Vector3 op, pp;
  op.setValue(origin.position.x, origin.position.y, origin.position.z);
  pp.setValue(pose.position.x, pose.position.y, pose.position.z);

  tf2::Matrix3x3 rotation(oq);

  auto ret = rotation * pp + op;
  return {static_cast<float>(ret.x()), static_cast<float>(ret.y()), static_cast<float>(ret.z())};
}

/**
 * @brief Convert predicted paths to the sequence of waypoints.
 *
 * @param origin Pose origin.
 * @param path Sequence of predicted poses.
 * @return std::vector<rerun::Vec3D>
 */
std::vector<rerun::Vec3D> to_waypoints(
  const geometry_msgs::msg::Pose & origin,
  const autoware_perception_msgs::msg::PredictedPath & path)
{
  std::vector<rerun::Vec3D> waypoints;
  for (const auto & pose : path.path) {
    // waypoints.emplace_back(do_transform(origin, pose));
    waypoints.emplace_back(rerun::Vec3D(pose.position.x, pose.position.y, pose.position.z));
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
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  const auto entity_path = property_.entity(msg->header.frame_id);
  if (!entity_path) {
    stream_->log(property_.topic(), rerun::TextLog("There is no corresponding entity path"));
    return;
  }

  std::vector<rerun::LineStrip3D> linestrips;
  std::vector<rerun::components::ClassId> class_ids;
  for (const auto & object : msg->objects) {
    const auto & origin = object.kinematics.initial_pose_with_covariance.pose;
    const auto class_id = static_cast<uint16_t>(object.classification.front().label);

    for (const auto & path : object.kinematics.predicted_paths) {
      const auto waypoints = to_waypoints(origin, path);
      linestrips.emplace_back(rerun::LineStrip3D(waypoints));

      class_ids.emplace_back(class_id);
    }
  }

  stream_->log(entity_path.value(), rerun::LineStrips3D(linestrips).with_class_ids(class_ids));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::PredictedObjectsDisplay, awviz_common::Display);
