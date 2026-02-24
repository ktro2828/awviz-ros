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

#include "awviz_plugin/autoware_map/conversion.hpp"

#include "autoware_lanelet2_extension/utility/query.hpp"

namespace awviz_plugin
{
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

LaneletLineStrips convert_road_lanelets(
  const lanelet::LaneletLayer & layer, const lanelet::ConstLanelets & lanelets)
{
  static const std::set<std::string> attributes{"dashed", "solid", "dashed_dashed", "virtual"};

  LaneletLineStrips output;

  const auto road_lanelets = lanelet::utils::query::roadLanelets(lanelets);
  for (const auto & lanelet : road_lanelets) {
    const auto left_bound = lanelet.leftBound();
    const auto right_bound = lanelet.rightBound();

    const auto left_candidates = layer.findUsages(left_bound);
    for (const auto & candidate : left_candidates) {
      if (candidate == lanelet) {
        continue;  // exclude self lanelet
      } else if (candidate.leftBound() != right_bound && candidate.rightBound() != left_bound) {
        continue;  // exclude unshared lanelet
      }

      if (has_attribute(left_bound, attributes)) {
        std::vector<rerun::Position3D> points;
        points.reserve(left_bound.size());
        for (const auto & bound : left_bound) {
          points.emplace_back(rerun::Position3D(bound.x(), bound.y(), bound.z()));
        }
        if (!points.empty()) {
          output.emplace_back(points);
        }
      }
    }

    const auto right_candidates = layer.findUsages(right_bound);
    for (const auto & candidate : right_candidates) {
      if (candidate == lanelet) {
        continue;  // exclude self lanelet
      } else if (candidate.leftBound() != right_bound && candidate.rightBound() != left_bound) {
        continue;  // exclude unshared lanelet
      }

      if (has_attribute(right_bound, attributes)) {
        std::vector<rerun::Position3D> points;
        points.reserve(right_bound.size());
        for (const auto & bound : right_bound) {
          points.emplace_back(rerun::Position3D(bound.x(), bound.y(), bound.z()));
        }
        if (!points.empty()) {
          output.emplace_back(points);
        }
      }
    }
  }
  return output;
}

LaneletLineStrips convert_road_boundaries(const lanelet::ConstLanelets & lanelets)
{
  static const std::set<std::string> attributes{"road_border"};

  LaneletLineStrips output;

  const auto road_lanelets = lanelet::utils::query::roadLanelets(lanelets);
  for (const auto & lanelet : road_lanelets) {
    const auto left_bound = lanelet.leftBound();
    const auto right_bound = lanelet.rightBound();

    if (!left_bound.empty() && has_attribute(left_bound, attributes)) {
      std::vector<rerun::Position3D> points;
      points.reserve(left_bound.size());
      for (const auto & bound : left_bound) {
        points.emplace_back(rerun::Position3D(bound.x(), bound.y(), bound.z()));
      }
      if (!points.empty()) {
        output.emplace_back(points);
      }
    }

    if (!right_bound.empty() && has_attribute(right_bound, attributes)) {
      std::vector<rerun::Position3D> points;
      points.reserve(right_bound.size());
      for (const auto & bound : right_bound) {
        points.emplace_back(rerun::Position3D(bound.x(), bound.y(), bound.z()));
      }
      if (!points.empty()) {
        output.emplace_back(points);
      }
    }
  }

  return output;
}

LaneletLineStrips convert_centerlines(const lanelet::ConstLanelets & lanelets)
{
  LaneletLineStrips output;

  const auto road_lanelets = lanelet::utils::query::roadLanelets(lanelets);
  for (const auto & lanelet : road_lanelets) {
    const auto centerline = lanelet.centerline();
    if (centerline.empty()) {
      continue;
    }
    std::vector<rerun::Position3D> points;
    points.reserve(centerline.size());
    for (const auto & point : centerline) {
      points.emplace_back(rerun::Position3D(point.x(), point.y(), point.z()));
    }
    if (!points.empty()) {
      output.emplace_back(points);
    }
  }

  return output;
}

LaneletLineStrips convert_stop_lines(const lanelet::LineStringLayer & layer)
{
  LaneletLineStrips output;
  static const std::set<std::string> attributes{"stop_line", "stopline"};

  for (const auto & linestring : layer) {
    if (!has_attribute(linestring, attributes)) {
      continue;
    }
    std::vector<rerun::Position3D> points;
    points.reserve(linestring.size());
    for (const auto & point : linestring) {
      points.emplace_back(rerun::Position3D(point.x(), point.y(), point.z()));
    }
    if (!points.empty()) {
      output.emplace_back(points);
    }
  }

  return output;
}

std::vector<LaneletLineStrips> convert_crosswalks(const lanelet::ConstLanelets & lanelets)
{
  std::vector<LaneletLineStrips> output;

  const auto crosswalks = lanelet::utils::query::crosswalkLanelets(lanelets);
  for (const auto & lanelet : crosswalks) {
    std::vector<rerun::Position3D> points;
    for (const auto & point : lanelet.polygon3d()) {
      points.emplace_back(rerun::Position3D(point.x(), point.y(), point.z()));
    }
    if (!points.empty()) {
      points.emplace_back(points.front());
      LaneletLineStrips current;
      current.emplace_back(points);
      output.emplace_back(current);
    }
  }

  return output;
}
}  // namespace awviz_plugin
