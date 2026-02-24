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

#include "awviz_plugin/autoware_planning/trajectory_display.hpp"

#include <rerun.hpp>
#include <rerun/archetypes/mesh3d.hpp>

#include <cmath>
#include <vector>

namespace awviz_plugin
{
namespace
{

rerun::Mesh3D to_ribbon_mesh(
  const autoware_planning_msgs::msg::Trajectory & trajectory, float width)
{
  const float half_width = width * 0.5f;

  std::vector<rerun::Position3D> vertices;
  std::vector<rerun::components::TriangleIndices> triangles;

  vertices.reserve(trajectory.points.size() * 2);
  triangles.reserve((trajectory.points.size() - 1) * 2);

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    const auto & pose = point.pose;
    const auto & q = pose.orientation;

    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    const float dx = static_cast<float>(-std::sin(yaw) * half_width);
    const float dy = static_cast<float>(std::cos(yaw) * half_width);

    const float x = static_cast<float>(pose.position.x);
    const float y = static_cast<float>(pose.position.y);
    const float z = static_cast<float>(pose.position.z);

    vertices.emplace_back(x + dx, y + dy, z);
    vertices.emplace_back(x - dx, y - dy, z);

    if (i > 0) {
      const uint32_t prev_left = static_cast<uint32_t>((i - 1) * 2);
      const uint32_t prev_right = prev_left + 1;
      const uint32_t curr_left = static_cast<uint32_t>(i * 2);
      const uint32_t curr_right = curr_left + 1;

      triangles.emplace_back(prev_left, prev_right, curr_right);
      triangles.emplace_back(prev_left, curr_right, curr_left);
    }
  }

  // light purple
  return rerun::Mesh3D(vertices).with_triangle_indices(triangles).with_albedo_factor(
    rerun::components::AlbedoFactor(rerun::datatypes::Rgba32(0x50, 0x3C, 0xC8, 0x00)));
}
}  // namespace

TrajectoryDisplay::TrajectoryDisplay()
: awviz_common::RosTopicDisplay<autoware_planning_msgs::msg::Trajectory>()
{
}

void TrajectoryDisplay::log_message(autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  log_timestamp(rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec));

  const auto entity_path = resolve_entity_path(msg->header.frame_id);
  if (!entity_path) {
    log_warning_for_missing_entity(msg->header.frame_id);
    return;
  }

  if (msg->points.size() < 2) {
    log_warning_text("Trajectory has fewer than 2 points; cannot build ribbon mesh.");
    return;
  }

  constexpr auto mesh_width = 2.0f;
  const auto mesh = to_ribbon_mesh(*msg, mesh_width);
  stream_->log(entity_path.value(), mesh);
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::TrajectoryDisplay, awviz_common::Display);
