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

#ifndef AWVIZ_PLUGIN__AUTOWARE_MAP__CONVERTION_HPP_
#define AWVIZ_PLUGIN__AUTOWARE_MAP__CONVERTION_HPP_

#include <rerun.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <set>
#include <string>
#include <vector>

namespace awviz_plugin
{
/**
 * @brief A class stores a set of line strips, and creates a new `rerun::LineStrips3D` object.
 */
class LaneletLineStrips
{
public:
  LaneletLineStrips() = default;

  /**
   * @brief Append a new line strip to the end of the container.
   *
   * @param points Sequence of points in the line strip.
   */
  void emplace_back(const std::vector<rerun::Position3D> & points)
  {
    line_strips_.emplace_back(rerun::LineStrip3D(points));
  }

  /**
   * @brief Create a `rerun::LineStrips3D` object.
   *
   * @return New `rerun::LineStrips3D` object.
   */
  rerun::LineStrips3D as_linestrips() const { return rerun::LineStrips3D(line_strips_); }

  /**
   * @brief Access the underlying line strip list.
   *
   * @return Reference to the stored line strips.
   */
  const std::vector<rerun::LineStrip3D> & line_strips() const { return line_strips_; }

private:
  std::vector<rerun::LineStrip3D> line_strips_;  //!< Set of line strips.
};

/**
 * @brief Check if the input linestring has same attribute in the specfied set of attributes.
 *
 * @param linestring LineString object.
 * @param attributes Set of attributes.
 * @return Return true if the linestring has.
 */
bool has_attribute(
  const lanelet::ConstLineString3d & linestring, const std::set<std::string> & attributes);

/**
 * @brief Convert road lanelets to line strips.
 *
 * @param layer Lanelet layer.
 * @param lanelets Set of all lanelets associated with the vector map.
 * @return LaneletLineStrips object.
 */
LaneletLineStrips convert_road_lanelets(
  const lanelet::LaneletLayer & layer, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Convert road boundaries to line strips.
 *
 * @param all_lanelets Set of all lanelets associated with the vector map.
 * @return LaneletLineStrips object.
 */
LaneletLineStrips convert_road_boundaries(const lanelet::ConstLanelets & lanelets);

/**
 * @brief Convert centerlines to line strips.
 *
 * @param lanelets Set of all lanelets associated with the vector map.
 * @return LaneletLineStrips object.
 */
LaneletLineStrips convert_centerlines(const lanelet::ConstLanelets & lanelets);

/**
 * @brief Convert stop lines to line strips.
 *
 * @param layer LineString layer.
 * @return LaneletLineStrips object.
 */
LaneletLineStrips convert_stop_lines(const lanelet::LineStringLayer & layer);

/**
 * @brief Convert crosswalks to line strips.
 *
 * @param lanelets Set of all lanelets associated with the vector map.
 * @return Vector of LaneletLineStrips.
 */
std::vector<LaneletLineStrips> convert_crosswalks(const lanelet::ConstLanelets & lanelets);
}  // namespace awviz_plugin

#endif
