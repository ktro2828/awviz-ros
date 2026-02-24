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

namespace awviz_plugin
{
namespace
{
static const rerun::Color ROADLINE_COLOR(255, 215, 0);    // gold
static const rerun::Color BOUNDARY_COLOR(255, 255, 255);  // white
static const rerun::Color CENTERLINE_COLOR(0, 255, 255);  // cyan
static const rerun::Color STOPLINE_COLOR(255, 0, 0);      // red
static const rerun::Color CROSSWALK_COLOR(0, 255, 0);     // green
}  // namespace

LaneletDisplay::LaneletDisplay()
: awviz_common::RosTopicDisplay<autoware_map_msgs::msg::LaneletMapBin>()
{
}

void LaneletDisplay::log_message(autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  log_timestamp(rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec));

  const auto entity_path = resolve_entity_path(msg->header.frame_id);
  if (!entity_path) {
    warn_missing_entity(msg->header.frame_id);
    return;
  }

  // Load LaneletMap
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);

  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);

  std::vector<rerun::LineStrip3D> all_strips;
  std::vector<rerun::Color> all_colors;

  const auto append_strips =
    [&](const std::vector<rerun::LineStrip3D> & strips, const rerun::Color & color) {
      if (strips.empty()) {
        return;
      }
      all_strips.insert(all_strips.end(), strips.begin(), strips.end());
      all_colors.insert(all_colors.end(), strips.size(), color);
    };

  {
    // 1. all road lanelets to a set of line strips
    const auto road_lines = convert_road_lanelets(lanelet_map_->laneletLayer, all_lanelets);
    append_strips(road_lines.line_strips(), ROADLINE_COLOR);
  }

  {
    // 2. road boundaries
    const auto boundary_lines = convert_road_boundaries(all_lanelets);
    append_strips(boundary_lines.line_strips(), BOUNDARY_COLOR);
  }

  {
    // 3. centerlines
    const auto center_lines = convert_centerlines(all_lanelets);
    append_strips(center_lines.line_strips(), CENTERLINE_COLOR);
  }

  {
    // 4. stop lines
    const auto stop_lines = convert_stop_lines(lanelet_map_->lineStringLayer);
    append_strips(stop_lines.line_strips(), STOPLINE_COLOR);
  }

  {
    // 5. all crosswalk lanelets to a set of line strips
    const auto crosswalk_lines = convert_crosswalks(all_lanelets);
    for (const auto & lines : crosswalk_lines) {
      append_strips(lines.line_strips(), CROSSWALK_COLOR);
    }
  }

  stream_->log(entity_path.value(), rerun::LineStrips3D(all_strips).with_colors(all_colors));
}
}  // namespace awviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awviz_plugin::LaneletDisplay, awviz_common::Display);
