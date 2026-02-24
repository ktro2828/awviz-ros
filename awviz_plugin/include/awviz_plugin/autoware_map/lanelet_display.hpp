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

#include "awviz_plugin/autoware_map/conversion.hpp"

#include <awviz_common/display.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <memory>
#include <vector>

namespace awviz_plugin
{
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
  lanelet::LaneletMapPtr lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
};
}  // namespace awviz_plugin
#endif  // AWVIZ_PLUGIN__AUTOWARE_MAP__LANELET_DISPLAY_HPP_
