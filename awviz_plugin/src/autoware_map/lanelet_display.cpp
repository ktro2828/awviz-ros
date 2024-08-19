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

#include "awviz_plugin/autoware_map/lanelet_display.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <set>

namespace
{
/**
 * @brief Check if the input linestring has same attribute in the specfied set of attributes.
 *
 * @param linestring LineString object.
 * @param attributes Set of attributes.
 * @return Return true if the linestring has.
 */
bool has_attribute(
  const lanelet::ConstLineString3d & linestring, const std::set<std::string> & attributes)
{
  if (!linestring.hasAttribute(lanelet::AttributeName::Type)) {
    return false;
  } else {
    const auto & attr = linestring.attribute(lanelet::AttributeName::Type);
    return attributes.count(attr.value()) > 0;
  }
}
}  // namespace

namespace awviz_plugin
{
LaneletDisplay::LaneletDisplay()
: awviz_common::RosTopicDisplay<autoware_map_msgs::msg::LaneletMapBin>()
{
}

void LaneletDisplay::log_message(autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  stream_->set_time_seconds(
    TIMELINE_NAME, rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec).seconds());

  const auto entity_path = property_.entity(msg->header.frame_id);
  if (!entity_path) {
    stream_->log(property_.topic(), rerun::TextLog("There is no corresponding entity path"));
    return;
  }

  // Load LaneletMap
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);

  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);

  std::vector<rerun::Mesh3D> all_meshes;
  {
    // 1. all road lanelets to a single mesh
    const auto road_mesh = convert_road_lanelets(all_lanelets);
    all_meshes.emplace_back(road_mesh.as_mesh());
  }

  {
    // 1. all crosswalk lanelets to a set of mesh
    const auto crosswalk_meshes = convert_crosswalks(all_lanelets);
    for (const auto & mesh : crosswalk_meshes) {
      all_meshes.emplace_back(mesh.as_mesh());
    }
  }

  stream_->log(entity_path.value(), all_meshes);
}

LaneletMesh LaneletDisplay::convert_road_lanelets(const lanelet::ConstLanelets & all_lanelets) const
{
  static const std::set<std::string> attributes{"line_thin", "line_thick"};

  LaneletMesh output;

  const auto road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  for (const auto & lanelet : road_lanelets) {
    const auto left_bound = lanelet.leftBound();
    const auto right_bound = lanelet.rightBound();

    const auto left_candidates = lanelet_map_->laneletLayer.findUsages(left_bound);
    for (const auto & candidate : left_candidates) {
      if (candidate == lanelet) {
        continue;  // exclude self lanelet
      } else if (candidate.leftBound() != right_bound && candidate.rightBound() != left_bound) {
        continue;  // exclude unshared lanelet
      }

      if (has_attribute(left_bound, attributes)) {
        for (const auto & bound : left_bound) {
          output.emplace_back(rerun::Position3D(bound.x(), bound.y(), bound.z()));
        }
      }
    }

    const auto right_candidates = lanelet_map_->laneletLayer.findUsages(right_bound);
    for (const auto & candidate : right_candidates) {
      if (candidate == lanelet) {
        continue;  // exclude self lanelet
      } else if (candidate.leftBound() != right_bound && candidate.rightBound() != left_bound) {
        continue;  // exclude unshared lanelet
      }

      if (has_attribute(right_bound, attributes)) {
        for (const auto & bound : right_bound) {
          output.emplace_back(rerun::Position3D(bound.x(), bound.y(), bound.z()));
        }
      }
    }
  }
  return output;
}

std::vector<LaneletMesh> LaneletDisplay::convert_crosswalks(
  const lanelet::ConstLanelets & all_lanelets) const
{
  std::vector<LaneletMesh> output;

  const auto crosswalks = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  for (const auto & lanelet : crosswalks) {
    LaneletMesh current;
    for (const auto & point : lanelet.polygon3d()) {
      current.emplace_back(rerun::Position3D(point.x(), point.y(), point.z()));
    }
    output.emplace_back(current);
  }

  return output;
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::LaneletDisplay, awviz_common::Display);
