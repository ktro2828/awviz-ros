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

#ifndef AWVIZ_PLUGIN__AUTOWARE_MAP__LANELET_DISPLAY_HPP_
#define AWVIZ_PLUGIN__AUTOWARE_MAP__LANELET_DISPLAY_HPP_

#include <awviz_common/display.hpp>
#include <rerun.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <vector>

namespace awviz_plugin
{
/**
 * @brief A class stores a set of vertices and normals, and creates a new `rerun::Mesh3D` object.
 */
class LaneletMesh
{
public:
  LaneletMesh() = default;

  /**
   * @brief Append new vertex and normal both to the end of the container.
   *
   * @param vertex Vertex element.
   * @param normal Vertex normal.
   */
  void emplace_back(const rerun::Position3D & vertex, const rerun::Vector3D & normal)
  {
    vertices_.emplace_back(vertex);
    normals_.emplace_back(normal);
  }

  /**
   * @brief Append a new vertex to the end of the container. Then, the normal is `{0.0f,
   * 0.0f, 1.0f}`.
   *
   * @param vertex Vertex element.
   */
  void emplace_back(const rerun::Position3D & vertex)
  {
    vertices_.emplace_back(vertex);
    normals_.emplace_back(0.f, 0.f, 1.0f);
  }

  /**
   * @brief Create a `rerun::Mesh3D` object with specifying normals.
   * @todo `rerun::Mesh3D` supports `::with_albedo_factor()` from `v0.18.0`.
   *
   * @return New `rerun::Mesh3D` object.
   */
  rerun::Mesh3D as_mesh() const { return rerun::Mesh3D(vertices_).with_vertex_normals(normals_); }

private:
  std::vector<rerun::Position3D> vertices_;  //!< Set of positions of each vertex.
  std::vector<rerun::Vector3D> normals_;     //!< Set of normals of each vertex.
};

/**
 * @brief A display plugin for `autoware_map_msgs::msg::LaneletMapBin`.
 */
class LaneletDisplay : public awviz_common::RosTopicDisplay<autoware_map_msgs::msg::LaneletMapBin>
{
public:
  /**
   * @brief Construct a new object.
   */
  LaneletDisplay();

protected:
  void log_message(typename autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) override;

private:
  /**
   * @brief Convert lanelets of roads to a single triangle mesh.
   *
   * @param lanelets Set of all lanelets associated with any roads.
   */
  LaneletMesh convert_road_lanelets(const lanelet::ConstLanelets & road_lanelets) const;

  /**
   * @brief Convert crosswalks to a set of triangle mesh.
   *
   * @param all_lanelets Set of all lanelets associated with the vector map.
   * @param vertices Output container to store vertices.
   */
  std::vector<LaneletMesh> convert_crosswalks(const lanelet::ConstLanelets & all_lanelets) const;

private:
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ =
    std::make_shared<lanelet::LaneletMap>();  //!< Shared pointer stores `lanelet::LaneletMap`.
};
}  // namespace awviz_plugin
#endif  // AWVIZ_PLUGIN__AUTOWARE_MAP__LANELET_DISPLAY_HPP_
